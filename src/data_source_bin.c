#include "data_source.h"
#include "dataflash.h"
#include "map_session.h"
#include "ray_transform.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ArduPilot DataFlash replay.
//
// Like the tlog source, the log is decoded into the shared map once up front
// and scrubbing afterwards is timeline work. Ranging comes from RFND
// (rangefinders, one record per instance) and PRX (proximity, eight sectors);
// both are fed through exactly the same transform the live path uses.

#define BIN_PRX_SECTORS 8

typedef struct {
    char           path[1024];
    map_session_t *ms;
    int            slot;

    int64_t        first_ns;
    int64_t        last_ns;
    double         pos_s;

    // Trajectory replayed for `ds->state`, sampled from the pre-pass so seeking
    // does not mean re-parsing the file.
    struct { int64_t t_ns; int32_t lat, lon, alt; float q[4]; } *track;
    uint32_t       track_count, track_cap;
    uint32_t       track_cursor;

    // Rangefinder limits, picked up from PARM records when present.
    float          rngfnd_min_m[8];
    float          rngfnd_max_m[8];

    uint64_t       rays;
    uint64_t       records;
} bin_impl_t;

static void track_push(bin_impl_t *b, int64_t t_ns, double lat, double lon, double alt,
                       const float q[4]) {
    if (b->track_count == b->track_cap) {
        const uint32_t cap = b->track_cap ? b->track_cap * 2 : 4096;
        void *p = realloc(b->track, (size_t)cap * sizeof(*b->track));
        if (!p) return;
        b->track = p;
        b->track_cap = cap;
    }
    b->track[b->track_count].t_ns = t_ns;
    b->track[b->track_count].lat = (int32_t)lrint(lat * 1e7);
    b->track[b->track_count].lon = (int32_t)lrint(lon * 1e7);
    b->track[b->track_count].alt = (int32_t)lrint(alt * 1e3);
    memcpy(b->track[b->track_count].q, q, sizeof(float) * 4);
    b->track_count++;
}

static void handle_parm(bin_impl_t *b, const df_record_t *rec) {
    char name[32];
    double value = 0.0;
    if (!df_field_string(rec, "Name", name, sizeof(name))) return;
    if (!df_field_double(rec, "Value", &value)) return;

    // RNGFND<n>_MAX_CM / _MIN_CM, and the newer RNGFND<n>_MAX_M / _MIN_M.
    if (strncmp(name, "RNGFND", 6) != 0) return;
    const char *rest = name + 6;
    int instance = 0;
    while (*rest >= '0' && *rest <= '9') { instance = instance * 10 + (*rest - '0'); rest++; }
    if (instance < 1) instance = 1;
    const int idx = (instance - 1) % 8;

    if (strcmp(rest, "_MAX_CM") == 0)      b->rngfnd_max_m[idx] = (float)value * 0.01f;
    else if (strcmp(rest, "_MIN_CM") == 0) b->rngfnd_min_m[idx] = (float)value * 0.01f;
    else if (strcmp(rest, "_MAX_M") == 0)  b->rngfnd_max_m[idx] = (float)value;
    else if (strcmp(rest, "_MIN_M") == 0)  b->rngfnd_min_m[idx] = (float)value;
}

static void handle_rfnd(bin_impl_t *b, const df_record_t *rec, int64_t t_ns) {
    double dist = 0.0;
    int64_t instance = 0, orient = 25, status = 4;
    df_field_int(rec, "Instance", &instance);
    df_field_int(rec, "Orient", &orient);
    df_field_int(rec, "Stat", &status);

    // Newer logs store metres in `Dist`; older ones store centimetres in `Dist1`.
    bool have = df_field_double(rec, "Dist", &dist);
    if (!have) {
        if (df_field_double(rec, "Dist1", &dist)) { dist *= 0.01; have = true; }
    }
    if (!have) return;

    const int idx = (int)(instance % 8);
    const float max_m = b->rngfnd_max_m[idx] > 0.0f ? b->rngfnd_max_m[idx] : 40.0f;
    const float min_m = b->rngfnd_min_m[idx] > 0.0f ? b->rngfnd_min_m[idx] : 0.20f;

    ray_obs_t o;
    memset(&o, 0, sizeof(o));
    o.distance_m = (float)dist;
    o.min_distance_m = min_m;
    o.max_distance_m = max_m;
    o.orientation = (uint8_t)orient;
    o.covariance_cm2 = 255;
    // RangeFinder::Status: 0 NotConnected, 1 NoData, 2 OutOfRangeLow,
    // 3 OutOfRangeHigh, 4 Good. Out-of-range-high is a genuine no-return, so it
    // must reach the map as one rather than being dropped.
    if (status == 3) o.distance_m = max_m;
    else if (status != 4) return;

    map_session_feed_distance(b->ms, b->slot, t_ns, &o);
    b->rays++;
}

static void handle_prx(bin_impl_t *b, const df_record_t *rec, int64_t t_ns) {
    static const char *const k_labels[BIN_PRX_SECTORS] = {
        "D0", "D45", "D90", "D135", "D180", "D225", "D270", "D315"
    };
    obstacle_obs_t o;
    memset(&o, 0, sizeof(o));
    for (int i = 0; i < OBSTACLE_DISTANCE_SECTORS; i++) o.distances_cm[i] = UINT16_MAX;

    int found = 0;
    for (int i = 0; i < BIN_PRX_SECTORS; i++) {
        double d = 0.0;
        if (!df_field_double(rec, k_labels[i], &d)) continue;
        if (!(d >= 0.0) || !isfinite(d)) continue;
        double cm = d * 100.0;
        if (cm > 65534.0) cm = 65534.0;
        o.distances_cm[i] = (uint16_t)lrint(cm);
        found++;
    }
    if (found == 0) return;

    o.sector_count = BIN_PRX_SECTORS;
    o.frame = RT_FRAME_BODY_FRD;
    o.increment_deg = 45.0f;
    o.angle_offset_deg = 0.0f;
    o.min_distance_cm = 20;
    o.max_distance_cm = 4000;

    map_session_feed_obstacle(b->ms, b->slot, t_ns, &o);
    b->rays += BIN_PRX_SECTORS;
}

static int prescan(data_source_t *ds, bin_impl_t *b) {
    df_reader_t r;
    if (df_reader_open(&r, b->path) != 0) return -1;

    float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    double lat = 0.0, lon = 0.0, alt = 0.0;
    bool have_pos = false, have_att = false, span = false;
    int64_t t_ns = 0;

    df_record_t rec;
    while (df_reader_next(&r, &rec) == 1) {
        b->records++;
        int64_t rec_ns;
        if (df_record_time_ns(&rec, &rec_ns)) {
            t_ns = rec_ns;
            if (!span) { b->first_ns = t_ns; span = true; }
            b->last_ns = t_ns;
        }

        if (df_record_is(&rec, "PARM")) { handle_parm(b, &rec); continue; }

        if (df_record_is(&rec, "ORGN")) {
            double olat, olon, oalt;
            if (df_field_double(&rec, "Lat", &olat) &&
                df_field_double(&rec, "Lng", &olon)) {
                if (!df_field_double(&rec, "Alt", &oalt)) oalt = 0.0;
                map_session_feed_origin(b->ms, b->slot, FLEET_ORIGIN_SRC_GPS_ORIGIN,
                                        olat, olon, oalt);
            }
            continue;
        }

        if (df_record_is(&rec, "ATT")) {
            double roll = 0.0, pitch = 0.0, yaw = 0.0;
            df_field_double(&rec, "Roll", &roll);
            df_field_double(&rec, "Pitch", &pitch);
            df_field_double(&rec, "Yaw", &yaw);
            rt_quat_from_euler((float)(roll * M_PI / 180.0),
                               (float)(pitch * M_PI / 180.0),
                               (float)(yaw * M_PI / 180.0), q);
            have_att = true;
            map_session_feed_attitude(b->ms, b->slot, t_ns, q);
            if (have_pos) track_push(b, t_ns, lat, lon, alt, q);
            continue;
        }

        if (df_record_is(&rec, "POS") || df_record_is(&rec, "GPS")) {
            double plat, plon, palt;
            if (!df_field_double(&rec, "Lat", &plat)) continue;
            if (!df_field_double(&rec, "Lng", &plon)) continue;
            if (!df_field_double(&rec, "Alt", &palt)) palt = 0.0;
            // GPS records also carry a status; a 2D fix is not a position we
            // want anchoring the fleet frame.
            int64_t status = 3;
            if (df_record_is(&rec, "GPS") && df_field_int(&rec, "Status", &status) && status < 3)
                continue;
            lat = plat; lon = plon; alt = palt;
            have_pos = true;
            map_session_feed_global(b->ms, b->slot, t_ns, lat, lon, alt, NULL);
            if (have_att) track_push(b, t_ns, lat, lon, alt, q);
            continue;
        }

        if (df_record_is(&rec, "RFND")) { handle_rfnd(b, &rec, t_ns); continue; }
        if (df_record_is(&rec, "PRX"))  { handle_prx(b, &rec, t_ns); continue; }

        if (df_record_is(&rec, "MSG")) {
            char text[80];
            if (df_field_string(&rec, "Message", text, sizeof(text)))
                map_session_feed_event(b->ms, b->slot, t_ns, TL_EVENT_STATUSTEXT, 6, 0, text);
            continue;
        }
        if (df_record_is(&rec, "MODE")) {
            int64_t mode = 0;
            df_field_int(&rec, "ModeNum", &mode);
            map_session_feed_event(b->ms, b->slot, t_ns, TL_EVENT_MODE_CHANGE, 6,
                                   (uint32_t)mode, "mode change");
            continue;
        }
        if (df_record_is(&rec, "EV")) {
            int64_t id = 0;
            df_field_int(&rec, "Id", &id);
            map_session_feed_event(b->ms, b->slot, t_ns, TL_EVENT_SYSTEM_EVENT, 6,
                                   (uint32_t)id, NULL);
            continue;
        }
        // See the note in data_source_tlog.c: the queue sheds under pressure,
        // which is right for a live link and wrong for a pre-pass that nothing
        // is racing. Drain at a watermark so no ray is ever shed here.
        if (b->ms->ingest.stats.queue_depth >= 16384u)
            map_ingest_drain_all(&b->ms->ingest, &b->ms->map);
    }
    df_reader_close(&r);

    map_ingest_drain_all(&b->ms->ingest, &b->ms->map);
    b->ms->timeline.map_cursor = b->ms->timeline.ray_total;
    b->ms->timeline.map_state_ns = b->ms->timeline.head_ns;
    b->ms->timeline.map_valid = true;

    // DataFlash TimeUS is boot-relative. Unless a GPS week/ms pair upgraded it,
    // say so rather than implying the fleet is aligned.
    if (b->slot >= 0 && b->slot < MS_MAX_VEHICLES) {
        map_vehicle_t *v = &b->ms->veh[b->slot];
        if (v->tb.provenance == TIME_PROV_NONE) v->tb.provenance = TIME_PROV_BOOT_ASSUMED;
        ds->time_provenance = (int)v->tb.provenance;
        ds->time_offset_ns = v->tb.offset_ns;
    }
    return span ? 0 : -1;
}

static void apply_track(data_source_t *ds, bin_impl_t *b, int64_t t_ns) {
    if (b->track_count == 0) return;
    uint32_t i = b->track_cursor;
    if (i >= b->track_count || b->track[i].t_ns > t_ns) i = 0;
    while (i + 1 < b->track_count && b->track[i + 1].t_ns <= t_ns) i++;
    b->track_cursor = i;

    ds->state.lat = b->track[i].lat;
    ds->state.lon = b->track[i].lon;
    ds->state.alt = b->track[i].alt;
    memcpy(ds->state.quaternion, b->track[i].q, sizeof(ds->state.quaternion));
    ds->state.time_usec = (uint64_t)(b->track[i].t_ns / 1000);
    ds->state.valid = true;
}

static void bin_poll(data_source_t *ds, float dt) {
    bin_impl_t *b = (bin_impl_t *)ds->impl;

    if (!ds->playback.paused) {
        b->pos_s += (double)dt * (double)ds->playback.speed;
        if (b->pos_s > ds->playback.duration_s) {
            if (ds->playback.looping) b->pos_s = 0.0;
            else { b->pos_s = ds->playback.duration_s; ds->connected = false; }
        } else {
            ds->connected = true;
        }
    }
    apply_track(ds, b, b->first_ns + (int64_t)(b->pos_s * 1e9));

    ds->playback.position_s = (float)b->pos_s;
    if (ds->playback.duration_s > 0.0f)
        ds->playback.progress = ds->playback.position_s / ds->playback.duration_s;
}

static void bin_seek(data_source_t *ds, float target_s) {
    bin_impl_t *b = (bin_impl_t *)ds->impl;
    if (target_s < 0.0f) target_s = 0.0f;
    if (target_s > ds->playback.duration_s) target_s = ds->playback.duration_s;
    b->pos_s = (double)target_s;
    b->track_cursor = 0;
    apply_track(ds, b, b->first_ns + (int64_t)(b->pos_s * 1e9));
    ds->connected = true;
    ds->playback.position_s = (float)b->pos_s;
    if (ds->playback.duration_s > 0.0f)
        ds->playback.progress = ds->playback.position_s / ds->playback.duration_s;
}

static void bin_set_time_offset(data_source_t *ds, double offset_s) {
    bin_impl_t *b = (bin_impl_t *)ds->impl;
    ds->playback.time_offset_s = (float)offset_s;
    if (b->ms && b->slot >= 0 && b->slot < MS_MAX_VEHICLES) {
        timebase_set_manual(&b->ms->veh[b->slot].tb, (int64_t)(offset_s * 1e9));
        ds->time_provenance = (int)b->ms->veh[b->slot].tb.provenance;
        ds->time_offset_ns = b->ms->veh[b->slot].tb.offset_ns;
    }
}

static void bin_close(data_source_t *ds) {
    bin_impl_t *b = (bin_impl_t *)ds->impl;
    if (!b) return;
    free(b->track);
    free(b);
    ds->impl = NULL;
}

static const data_source_ops_t bin_ops = {
    .poll = bin_poll,
    .seek = bin_seek,
    .set_time_offset = bin_set_time_offset,
    .close = bin_close,
};

int data_source_bin_create(data_source_t *ds, const char *path,
                           struct map_session *ms, int slot) {
    if (!ds || !path || !ms) return -1;
    memset(ds, 0, sizeof(*ds));
    ds->ops = &bin_ops;
    ds->map = ms;
    ds->map_slot = slot;

    bin_impl_t *b = (bin_impl_t *)calloc(1, sizeof(bin_impl_t));
    if (!b) return -1;
    snprintf(b->path, sizeof(b->path), "%s", path);
    b->ms = (map_session_t *)ms;
    b->slot = slot;

    ds->impl = b;
    if (prescan(ds, b) != 0) { free(b); ds->impl = NULL; return -1; }

    ds->connected = true;
    ds->sysid = (uint8_t)(slot < 0 ? 1 : slot + 1);
    ds->mav_type = 2;   // MAV_TYPE_QUADROTOR; DataFlash has no direct equivalent
    ds->playback.speed = 1.0f;
    ds->playback.interpolation = true;
    ds->playback.correlation = NAN;
    ds->playback.rmse = NAN;
    ds->playback.duration_s = (float)((double)(b->last_ns - b->first_ns) / 1e9);
    if (b->track_count) {
        ds->home.lat = b->track[0].lat;
        ds->home.lon = b->track[0].lon;
        ds->home.alt = b->track[0].alt;
        ds->home.valid = true;
    }
    return 0;
}

uint64_t data_source_bin_rays(const data_source_t *ds) {
    const bin_impl_t *b = (const bin_impl_t *)ds->impl;
    return b ? b->rays : 0;
}
