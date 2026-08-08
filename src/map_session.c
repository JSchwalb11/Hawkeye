#include "map_session.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void map_session_config_defaults(map_session_config_t *cfg) {
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->max_vehicles = 16;
    cfg->origin_policy = FLEET_ORIGIN_FIRST_SEEN;
}

int map_session_init(map_session_t *ms, const map_session_config_t *cfg) {
    if (!ms) return -1;
    map_session_config_t d;
    map_session_config_defaults(&d);
    if (cfg) d = *cfg;
    if (d.max_vehicles <= 0) d.max_vehicles = 16;

    memset(ms, 0, sizeof(*ms));
    ms->focus_vehicle = -1;
    ms->solo_vehicle = -1;
    ms->enabled = true;

    octomap_config_t oc;
    memset(&oc, 0, sizeof(oc));
    oc.byte_cap = d.map_byte_cap;
    oc.root_size_m = d.root_size_m;
    oc.max_depth = d.max_depth;
    oc.coarse_depth = d.coarse_depth;
    oc.skip_near_m = d.skip_near_m;
    if (octomap_init(&ms->map, &oc) != 0) return -1;

    map_ingest_config_t ic;
    memset(&ic, 0, sizeof(ic));
    ic.queue_capacity = d.queue_capacity;
    ic.budget_per_drain = d.budget_per_drain;
    if (map_ingest_init(&ms->ingest, &ic) != 0) { octomap_free(&ms->map); return -1; }

    timeline_config_t tc;
    memset(&tc, 0, sizeof(tc));
    tc.ray_log_bytes = d.ray_log_bytes;
    tc.keyframe_bytes = d.keyframe_bytes;
    tc.keyframe_interval_s = d.keyframe_interval_s;
    tc.max_vehicles = d.max_vehicles;
    if (timeline_init(&ms->timeline, &tc) != 0) {
        map_ingest_free(&ms->ingest);
        octomap_free(&ms->map);
        return -1;
    }

    fleet_frame_init(&ms->frame, d.origin_policy);
    timebase_session_init(&ms->clock);
    for (int i = 0; i < MS_MAX_VEHICLES; i++) timebase_init(&ms->veh[i].tb);
    return 0;
}

void map_session_free(map_session_t *ms) {
    if (!ms) return;
    timeline_free(&ms->timeline);
    map_ingest_free(&ms->ingest);
    octomap_free(&ms->map);
    memset(ms, 0, sizeof(*ms));
}

// ------------------------------------------------------------ slots

void map_session_bind_slot(map_session_t *ms, int slot, uint8_t sysid) {
    if (!ms || slot < 0 || slot >= MS_MAX_VEHICLES) return;
    if (!ms->veh[slot].present) {
        ms->veh[slot].present = true;
        if (slot + 1 > ms->vehicle_count) ms->vehicle_count = slot + 1;
    }
    ms->veh[slot].sysid = sysid;
}

int map_session_slot_for_sysid(map_session_t *ms, uint8_t sysid) {
    if (!ms) return -1;
    for (int i = 0; i < MS_MAX_VEHICLES; i++)
        if (ms->veh[i].present && ms->veh[i].sysid == sysid) return i;
    for (int i = 0; i < MS_MAX_VEHICLES; i++) {
        if (!ms->veh[i].present) {
            map_session_bind_slot(ms, i, sysid);
            return i;
        }
    }
    return -1;
}

// ------------------------------------------------------------ feeds

static void publish_sample(map_session_t *ms, int slot, int64_t t_ns,
                           const float vel_ned[3]) {
    map_vehicle_t *v = &ms->veh[slot];
    tl_sample_t s;
    memset(&s, 0, sizeof(s));
    s.t_ns = t_ns;
    s.enu[0] = v->enu[0]; s.enu[1] = v->enu[1]; s.enu[2] = v->enu[2];
    memcpy(s.q, v->q_ned_body, sizeof(s.q));
    if (vel_ned) memcpy(s.vel_ned, vel_ned, sizeof(s.vel_ned));
    s.nav_state = v->have_mode ? (uint8_t)(v->custom_mode & 0xFF) : 0xFF;
    s.flags = (uint8_t)((v->pos_valid ? TL_SAMPLE_POS_VALID : 0)
                      | (v->att_valid ? TL_SAMPLE_ATT_VALID : 0));
    timeline_add_sample(&ms->timeline, (uint8_t)slot, &s);
}

void map_session_feed_origin(map_session_t *ms, int slot, uint8_t src,
                             double lat_deg, double lon_deg, double alt_m) {
    if (!ms || slot < 0 || slot >= MS_MAX_VEHICLES) return;
    fleet_frame_note_origin(&ms->frame, (uint8_t)slot, src, lat_deg, lon_deg, alt_m);
}

void map_session_feed_global(map_session_t *ms, int slot, int64_t t_ns,
                             double lat_deg, double lon_deg, double alt_m,
                             const float vel_ned[3]) {
    if (!ms || slot < 0 || slot >= MS_MAX_VEHICLES) return;
    map_vehicle_t *v = &ms->veh[slot];
    v->lat_deg = lat_deg; v->lon_deg = lon_deg; v->alt_m = alt_m;
    v->global_valid = true;

    // A vehicle that never announces an origin still needs one, or its local
    // NED reports have nothing to hang off.
    if (!ms->frame.vehicle[slot].valid)
        fleet_frame_note_origin(&ms->frame, (uint8_t)slot, FLEET_ORIGIN_SRC_FIRST_FIX,
                                lat_deg, lon_deg, alt_m);

    double enu[3];
    if (fleet_frame_global_to_enu(&ms->frame, lat_deg, lon_deg, alt_m, enu)) {
        memcpy(v->enu, enu, sizeof(v->enu));
        v->pos_valid = true;
        v->pos_t_ns = t_ns;
        publish_sample(ms, slot, t_ns, vel_ned);
    }
}

void map_session_feed_local_ned(map_session_t *ms, int slot, int64_t t_ns,
                                const double ned[3], const float vel_ned[3]) {
    if (!ms || slot < 0 || slot >= MS_MAX_VEHICLES || !ned) return;
    map_vehicle_t *v = &ms->veh[slot];
    double enu[3];
    if (!fleet_frame_local_ned_to_enu(&ms->frame, (uint8_t)slot, ned, enu)) return;
    memcpy(v->enu, enu, sizeof(v->enu));
    v->pos_valid = true;
    v->pos_t_ns = t_ns;
    publish_sample(ms, slot, t_ns, vel_ned);
}

void map_session_feed_attitude(map_session_t *ms, int slot, int64_t t_ns,
                               const float q_ned_body[4]) {
    if (!ms || slot < 0 || slot >= MS_MAX_VEHICLES || !q_ned_body) return;
    map_vehicle_t *v = &ms->veh[slot];
    memcpy(v->q_ned_body, q_ned_body, sizeof(v->q_ned_body));
    rt_quat_normalize(v->q_ned_body);
    v->att_valid = true;
    v->att_t_ns = t_ns;
    publish_sample(ms, slot, t_ns, NULL);
}

void map_session_feed_event(map_session_t *ms, int slot, int64_t t_ns,
                            tl_event_kind_t kind, uint8_t severity,
                            uint32_t code, const char *text) {
    if (!ms) return;
    tl_event_t e;
    memset(&e, 0, sizeof(e));
    e.t_ns = t_ns;
    e.vehicle_id = (uint8_t)(slot < 0 ? 0 : slot);
    e.kind = (uint8_t)kind;
    e.severity = severity;
    e.code = code;
    if (text) {
        strncpy(e.text, text, sizeof(e.text) - 1);
        e.text[sizeof(e.text) - 1] = '\0';
    }
    timeline_add_event(&ms->timeline, &e);
}

// Resolve the pose to pair with an observation stamped at t_ns. Interpolating
// the recorded track beats grabbing the latest attitude: a moving, yawing
// vehicle paired with a stale attitude smears a flat wall into a curve.
static bool pose_at(const map_session_t *ms, int slot, int64_t t_ns,
                    double enu[3], float q[4]) {
    tl_sample_t s;
    if (timeline_pose_at(&ms->timeline, (uint8_t)slot, t_ns, &s)) {
        if (s.flags & TL_SAMPLE_POS_VALID) {
            memcpy(enu, s.enu, sizeof(double) * 3);
            memcpy(q, s.q, sizeof(float) * 4);
            return true;
        }
    }
    const map_vehicle_t *v = &ms->veh[slot];
    if (!v->pos_valid) return false;
    memcpy(enu, v->enu, sizeof(double) * 3);
    memcpy(q, v->q_ned_body, sizeof(float) * 4);
    return true;
}

static void enqueue_rays(map_session_t *ms, int slot, int64_t t_ns,
                         const om_ray_t *rays, int count) {
    if (count <= 0) return;
    map_vehicle_t *v = &ms->veh[slot];

    if (!map_session_vehicle_contributes(ms, slot)) {
        v->rays_rejected += (uint64_t)count;
        return;
    }

    // Log first, so each ray has the lifetime index the queue will report its
    // progress in. The log is the record of what the map is built from, so it
    // has to be the thing everything else is expressed against.
    const uint64_t before = ms->ingest.stats.dropped;
    const uint64_t seq0 = timeline_add_ray(&ms->timeline, &rays[0], t_ns);
    for (int i = 1; i < count; i++) timeline_add_ray(&ms->timeline, &rays[i], t_ns);
    map_ingest_push_batch(&ms->ingest, rays, count, seq0);
    v->rays_contributed += (uint64_t)count;

    // Anything the queue shed never reached the map; mark it so a later
    // reconstruction does not invent evidence the operator never saw.
    uint64_t lost[64];
    uint32_t nlost = map_ingest_take_dropped(&ms->ingest, lost, 64);
    for (uint32_t i = 0; i < nlost; i++)
        timeline_mark_ray_skipped(&ms->timeline, lost[i]);

    const uint64_t shed = ms->ingest.stats.dropped - before;
    if (shed) timeline_note_drop(&ms->timeline, t_ns, (uint8_t)slot, (uint32_t)shed);
}

void map_session_feed_distance(map_session_t *ms, int slot, int64_t t_ns,
                               const ray_obs_t *obs) {
    if (!ms || !ms->enabled || slot < 0 || slot >= MS_MAX_VEHICLES || !obs) return;
    if (!fleet_frame_ready(&ms->frame)) { map_ingest_note_rejected(&ms->ingest); return; }
    fleet_frame_freeze(&ms->frame);

    ray_obs_t o = *obs;
    if (!pose_at(ms, slot, t_ns, o.origin_enu, o.att_ned_body)) {
        map_ingest_note_rejected(&ms->ingest);
        ms->veh[slot].rays_rejected++;
        return;
    }
    o.vehicle_id = (uint8_t)slot;
    o.time_ms = (uint32_t)(t_ns / 1000000LL);

    om_ray_t ray;
    if (!rt_build_ray(&o, &ray)) {
        map_ingest_note_rejected(&ms->ingest);
        ms->veh[slot].rays_rejected++;
        return;
    }
    enqueue_rays(ms, slot, t_ns, &ray, 1);
}

void map_session_feed_obstacle(map_session_t *ms, int slot, int64_t t_ns,
                               const obstacle_obs_t *obs) {
    if (!ms || !ms->enabled || slot < 0 || slot >= MS_MAX_VEHICLES || !obs) return;
    if (!fleet_frame_ready(&ms->frame)) { map_ingest_note_rejected(&ms->ingest); return; }
    fleet_frame_freeze(&ms->frame);

    obstacle_obs_t o = *obs;
    if (!pose_at(ms, slot, t_ns, o.origin_enu, o.att_ned_body)) {
        map_ingest_note_rejected(&ms->ingest);
        ms->veh[slot].rays_rejected++;
        return;
    }
    o.vehicle_id = (uint8_t)slot;
    o.time_ms = (uint32_t)(t_ns / 1000000LL);

    om_ray_t rays[OBSTACLE_DISTANCE_SECTORS];
    const int n = rt_expand_obstacle_distance(&o, rays, OBSTACLE_DISTANCE_SECTORS);
    enqueue_rays(ms, slot, t_ns, rays, n);
}

// ------------------------------------------------------------ per-frame

void map_session_tick(map_session_t *ms, float dt_s) {
    if (!ms) return;

    timeline_advance(&ms->timeline, dt_s);

    if (ms->timeline.playhead.pinned_to_head) {
        // Coming back to live after a scrub, the map is still showing the past.
        // Rebuild it to the head before resuming incremental insertion --
        // stamping the cursor instead would leave nothing to replay and freeze
        // the map there for the rest of the session.
        if (!ms->timeline.map_valid) {
            ms->timeline.playhead.t_ns = ms->timeline.head_ns;
            timeline_sync_map(&ms->timeline, &ms->map);
        }

        // Live: rays flow straight into the map and the map is the head state.
        map_ingest_drain(&ms->ingest, &ms->map, dt_s);
        // Report progress in the log's own terms. The drain is budgeted, so
        // assuming it consumed everything is how keyframes came to claim rays
        // that were still queued.
        if (ms->ingest.have_inserted)
            ms->timeline.map_cursor = ms->ingest.last_inserted_seq + 1;
        ms->timeline.map_state_ns = ms->timeline.head_ns;
        ms->timeline.map_valid = true;
        timeline_maybe_keyframe(&ms->timeline, &ms->map, ms->timeline.head_ns);
    } else {
        // Scrubbed: the map shows the past, so live rays must not be inserted
        // into it. They are already in the ray log, so the reconstruction (and
        // the rebuild on the way back to live) covers them.
        map_ingest_discard_all(&ms->ingest);
        timeline_sync_map(&ms->timeline, &ms->map);
    }
}

uint32_t map_session_resync(map_session_t *ms) {
    if (!ms) return 0;
    // Pending rays belong to the head, not to wherever the playhead is, so they
    // are dropped from the queue rather than inserted. The log still has them.
    map_ingest_discard_all(&ms->ingest);
    ms->timeline.map_valid = false;
    return timeline_sync_map(&ms->timeline, &ms->map);
}

// ------------------------------------------------------------ focus / mute

void map_session_set_focus(map_session_t *ms, int slot) {
    if (ms) ms->focus_vehicle = slot;
}

void map_session_set_mute(map_session_t *ms, int slot, bool muted) {
    if (!ms || slot < 0 || slot >= MS_MAX_VEHICLES) return;
    ms->veh[slot].muted = muted;
}

void map_session_set_solo(map_session_t *ms, int slot) {
    if (ms) ms->solo_vehicle = slot;
}

bool map_session_vehicle_contributes(const map_session_t *ms, int slot) {
    if (!ms || slot < 0 || slot >= MS_MAX_VEHICLES) return false;
    if (ms->veh[slot].muted) return false;
    if (ms->solo_vehicle >= 0 && ms->solo_vehicle != slot) return false;
    return true;
}

uint32_t map_session_focus_mask(const map_session_t *ms) {
    if (!ms || ms->focus_vehicle < 0) return 0;
    return om_vehicle_bit((uint8_t)ms->focus_vehicle);
}

// ------------------------------------------------------------ reporting

time_provenance_t map_session_time_provenance(const map_session_t *ms) {
    const timebase_t *tbs[MS_MAX_VEHICLES];
    int n = 0;
    for (int i = 0; i < MS_MAX_VEHICLES && n < MS_MAX_VEHICLES; i++)
        if (ms->veh[i].present) tbs[n++] = &ms->veh[i].tb;
    return timebase_fleet_worst(tbs, n);
}

int64_t map_session_time_spread_ns(const map_session_t *ms) {
    const timebase_t *tbs[MS_MAX_VEHICLES];
    int n = 0;
    for (int i = 0; i < MS_MAX_VEHICLES && n < MS_MAX_VEHICLES; i++)
        if (ms->veh[i].present) tbs[n++] = &ms->veh[i].tb;
    return timebase_fleet_spread_ns(tbs, n);
}
