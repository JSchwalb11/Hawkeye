#include "data_source.h"
#include "map_session.h"
#include "mavlink_map_decode.h"
#include "ray_transform.h"
#include "tlog.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <mavlink.h>

// tlog replay.
//
// When a map session is attached the whole log is decoded into it up front, so
// scrubbing afterwards is pure timeline work rather than a re-decode. Feeding
// the map again on every seek would duplicate rays in the history, which is the
// one thing the time-indexed map must never do.
//
// Vehicle state for the existing renderer is streamed separately from the same
// file, because that path does have to rewind.

typedef struct {
    tlog_reader_t reader;
    char     path[1024];
    uint8_t  channel;

    map_session_t *ms;
    int      slot;

    int64_t  first_ns;
    int64_t  last_ns;
    double   pos_s;

    // Streaming decode for `ds->state`.
    bool     pending;
    mavlink_message_t pending_msg;
    int64_t  pending_ns;
    uint8_t  primary_sysid;
    bool     have_primary;
    bool     hil_seen;
    bool     att_seen;
    bool     pos_seen;

    uint64_t frames_total;
    uint64_t rays_from_log;
} tlog_impl_t;

static void state_apply(data_source_t *ds, tlog_impl_t *t, const mavlink_message_t *msg) {
    if (!t->have_primary) {
        t->primary_sysid = msg->sysid;
        t->have_primary = true;
    }
    if (msg->sysid != t->primary_sysid) return;

    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(msg, &hb);
            ds->mav_type = hb.type;
            ds->sysid = msg->sysid;
            ds->playback.current_nav_state = (uint8_t)(hb.custom_mode & 0xFF);
            break;
        }
        case MAVLINK_MSG_ID_HOME_POSITION: {
            mavlink_home_position_t h;
            mavlink_msg_home_position_decode(msg, &h);
            ds->home.lat = h.latitude;
            ds->home.lon = h.longitude;
            ds->home.alt = h.altitude;
            ds->home.valid = true;
            break;
        }
        case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN: {
            if (ds->home.valid) break;
            mavlink_gps_global_origin_t o;
            mavlink_msg_gps_global_origin_decode(msg, &o);
            ds->home.lat = o.latitude;
            ds->home.lon = o.longitude;
            ds->home.alt = o.altitude;
            ds->home.valid = true;
            break;
        }
        case MAVLINK_MSG_ID_HIL_STATE_QUATERNION: {
            mavlink_hil_state_quaternion_t h;
            mavlink_msg_hil_state_quaternion_decode(msg, &h);
            memcpy(ds->state.quaternion, h.attitude_quaternion, sizeof(ds->state.quaternion));
            ds->state.lat = h.lat; ds->state.lon = h.lon; ds->state.alt = h.alt;
            ds->state.vx = h.vx; ds->state.vy = h.vy; ds->state.vz = h.vz;
            ds->state.ind_airspeed = h.ind_airspeed;
            ds->state.true_airspeed = h.true_airspeed;
            ds->state.time_usec = h.time_usec;
            ds->state.valid = true;
            t->hil_seen = true;
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            if (t->hil_seen) break;
            mavlink_attitude_t a;
            mavlink_msg_attitude_decode(msg, &a);
            rt_quat_from_euler(a.roll, a.pitch, a.yaw, ds->state.quaternion);
            t->att_seen = true;
            ds->state.valid = t->pos_seen;
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
            if (t->hil_seen) break;
            mavlink_attitude_quaternion_t a;
            mavlink_msg_attitude_quaternion_decode(msg, &a);
            ds->state.quaternion[0] = a.q1; ds->state.quaternion[1] = a.q2;
            ds->state.quaternion[2] = a.q3; ds->state.quaternion[3] = a.q4;
            t->att_seen = true;
            ds->state.valid = t->pos_seen;
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            if (t->hil_seen) break;
            mavlink_global_position_int_t p;
            mavlink_msg_global_position_int_decode(msg, &p);
            ds->state.lat = p.lat; ds->state.lon = p.lon; ds->state.alt = p.alt;
            ds->state.vx = p.vx; ds->state.vy = p.vy; ds->state.vz = p.vz;
            ds->state.time_usec = (uint64_t)p.time_boot_ms * 1000ULL;
            t->pos_seen = true;
            ds->state.valid = t->att_seen;
            break;
        }
        default: break;
    }
}

// Feed the whole log into the shared map exactly once.
static void prescan(data_source_t *ds, tlog_impl_t *t) {
    if (!t->ms) return;
    tlog_reader_t r;
    if (tlog_reader_open(&r, t->path, t->channel) != 0) return;

    mavlink_message_t msg;
    int64_t arrival = 0;
    const uint64_t rays_before = t->ms->timeline.ray_total;
    while (tlog_reader_next(&r, (struct __mavlink_message *)&msg, &arrival) == 1) {
        mavlink_map_decode(t->ms, t->slot, (struct __mavlink_message *)&msg, arrival);
        t->frames_total++;
    }
    t->rays_from_log = t->ms->timeline.ray_total - rays_before;
    tlog_reader_close(&r);

    // Everything queued during the pre-pass belongs in the map before the first
    // frame is drawn; there is no wall clock to pace against here.
    map_ingest_drain_all(&t->ms->ingest, &t->ms->map);
    t->ms->timeline.map_cursor = t->ms->timeline.ray_total;
    t->ms->timeline.map_state_ns = t->ms->timeline.head_ns;
    t->ms->timeline.map_valid = true;

    if (t->slot >= 0 && t->slot < MS_MAX_VEHICLES) {
        ds->time_provenance = (int)t->ms->veh[t->slot].tb.provenance;
        ds->time_offset_ns = t->ms->veh[t->slot].tb.offset_ns;
    }
}

static void stream_to(data_source_t *ds, tlog_impl_t *t, int64_t target_ns) {
    for (;;) {
        if (!t->pending) {
            const int rc = tlog_reader_next(&t->reader,
                (struct __mavlink_message *)&t->pending_msg, &t->pending_ns);
            if (rc != 1) { ds->connected = false; return; }
            t->pending = true;
        }
        if (t->pending_ns > target_ns) return;
        state_apply(ds, t, &t->pending_msg);
        t->pending = false;
    }
}

static void tlog_poll(data_source_t *ds, float dt) {
    tlog_impl_t *t = (tlog_impl_t *)ds->impl;

    if (!ds->playback.paused) {
        t->pos_s += (double)dt * (double)ds->playback.speed;
        if (t->pos_s > ds->playback.duration_s) {
            if (ds->playback.looping) {
                data_source_seek(ds, 0.0f);
            } else {
                t->pos_s = ds->playback.duration_s;
                ds->connected = false;
            }
        } else {
            ds->connected = true;
        }
    }

    stream_to(ds, t, t->first_ns + (int64_t)(t->pos_s * 1e9));

    ds->playback.position_s = (float)t->pos_s;
    if (ds->playback.duration_s > 0.0f)
        ds->playback.progress = ds->playback.position_s / ds->playback.duration_s;
}

static void tlog_seek(data_source_t *ds, float target_s) {
    tlog_impl_t *t = (tlog_impl_t *)ds->impl;
    if (target_s < 0.0f) target_s = 0.0f;
    if (target_s > ds->playback.duration_s) target_s = ds->playback.duration_s;

    tlog_reader_rewind(&t->reader);
    t->pending = false;
    t->hil_seen = t->att_seen = t->pos_seen = false;
    t->have_primary = false;
    memset(&ds->state, 0, sizeof(ds->state));
    t->pos_s = (double)target_s;

    stream_to(ds, t, t->first_ns + (int64_t)(t->pos_s * 1e9));
    ds->connected = true;
    ds->playback.position_s = (float)t->pos_s;
    if (ds->playback.duration_s > 0.0f)
        ds->playback.progress = ds->playback.position_s / ds->playback.duration_s;
}

static void tlog_set_time_offset(data_source_t *ds, double offset_s) {
    tlog_impl_t *t = (tlog_impl_t *)ds->impl;
    ds->playback.time_offset_s = (float)offset_s;
    if (t->ms && t->slot >= 0 && t->slot < MS_MAX_VEHICLES) {
        timebase_set_manual(&t->ms->veh[t->slot].tb, (int64_t)(offset_s * 1e9));
        ds->time_provenance = (int)t->ms->veh[t->slot].tb.provenance;
        ds->time_offset_ns = t->ms->veh[t->slot].tb.offset_ns;
    }
}

static void tlog_close(data_source_t *ds) {
    tlog_impl_t *t = (tlog_impl_t *)ds->impl;
    if (!t) return;
    tlog_reader_close(&t->reader);
    free(t);
    ds->impl = NULL;
}

static const data_source_ops_t tlog_ops = {
    .poll = tlog_poll,
    .seek = tlog_seek,
    .set_time_offset = tlog_set_time_offset,
    .close = tlog_close,
};

int data_source_tlog_create(data_source_t *ds, const char *path,
                            struct map_session *ms, int slot, uint8_t channel) {
    if (!ds || !path) return -1;
    memset(ds, 0, sizeof(*ds));
    ds->ops = &tlog_ops;
    ds->map_slot = slot;
    ds->map = ms;

    tlog_impl_t *t = (tlog_impl_t *)calloc(1, sizeof(tlog_impl_t));
    if (!t) return -1;
    snprintf(t->path, sizeof(t->path), "%s", path);
    t->channel = channel;
    t->ms = (map_session_t *)ms;
    t->slot = slot;

    if (tlog_reader_open(&t->reader, path, channel) != 0) { free(t); return -1; }
    if (!tlog_reader_span(&t->reader, &t->first_ns, &t->last_ns)) {
        tlog_reader_close(&t->reader);
        free(t);
        return -1;
    }
    tlog_reader_rewind(&t->reader);

    ds->impl = t;
    ds->connected = true;
    ds->sysid = 1;
    ds->playback.speed = 1.0f;
    ds->playback.interpolation = true;
    ds->playback.correlation = NAN;
    ds->playback.rmse = NAN;
    ds->playback.duration_s = (float)((double)(t->last_ns - t->first_ns) / 1e9);

    prescan(ds, t);
    return 0;
}

uint64_t data_source_tlog_rays(const data_source_t *ds) {
    const tlog_impl_t *t = (const tlog_impl_t *)ds->impl;
    return t ? t->rays_from_log : 0;
}

uint64_t data_source_tlog_frames(const data_source_t *ds) {
    const tlog_impl_t *t = (const tlog_impl_t *)ds->impl;
    return t ? t->frames_total : 0;
}
