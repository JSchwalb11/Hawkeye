#include "timeline.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *const k_event_names[TL_EVENT_KIND_COUNT] = {
    "STATUSTEXT", "MODE", "ACK", "MISSION", "EVENT", "RAY DROP", "TIME",
};

const char *tl_event_kind_name(tl_event_kind_t kind) {
    if (kind < 0 || kind >= TL_EVENT_KIND_COUNT) return "?";
    return k_event_names[kind];
}

// ---------------------------------------------------------------- setup

static uint32_t pow2_at_least(size_t n) {
    uint32_t p = 1;
    while ((size_t)p < n && p < (1u << 30)) p <<= 1;
    return p;
}

int timeline_init(timeline_t *tl, const timeline_config_t *cfg) {
    if (!tl) return -1;
    memset(tl, 0, sizeof(*tl));

    const size_t track_bytes = (cfg && cfg->track_bytes_per_vehicle)
        ? cfg->track_bytes_per_vehicle : (size_t)4 * 1024 * 1024;
    const size_t ray_bytes = (cfg && cfg->ray_log_bytes)
        ? cfg->ray_log_bytes : (size_t)64 * 1024 * 1024;
    const size_t kf_bytes = (cfg && cfg->keyframe_bytes)
        ? cfg->keyframe_bytes : (size_t)96 * 1024 * 1024;
    const uint32_t ev_cap = (cfg && cfg->event_capacity) ? cfg->event_capacity : 4096;

    tl->max_vehicles = (cfg && cfg->max_vehicles) ? cfg->max_vehicles : 16;
    if (tl->max_vehicles > TL_MAX_VEHICLES) tl->max_vehicles = TL_MAX_VEHICLES;
    tl->track_bytes = track_bytes;

    tl->event_cap = pow2_at_least(ev_cap);
    tl->events = (tl_event_t *)calloc(tl->event_cap, sizeof(tl_event_t));
    if (!tl->events) return -1;

    tl->ray_cap = pow2_at_least(ray_bytes / sizeof(tl_ray_t));
    tl->rays = (tl_ray_t *)calloc(tl->ray_cap, sizeof(tl_ray_t));
    if (!tl->rays) { free(tl->events); tl->events = NULL; return -1; }

    tl->kf_cap = 64;
    tl->keyframes = (tl_keyframe_t *)calloc(tl->kf_cap, sizeof(tl_keyframe_t));
    if (!tl->keyframes) { free(tl->events); free(tl->rays); tl->events = NULL; tl->rays = NULL; return -1; }
    tl->kf_bytes_cap = kf_bytes;
    tl->kf_interval_s = (cfg && cfg->keyframe_interval_s > 0.0f) ? cfg->keyframe_interval_s : 20.0f;
    tl->kf_last_ns = INT64_MIN;

    tl->playhead.pinned_to_head = true;
    tl->playhead.speed = 1.0f;
    tl->map_valid = true;
    return 0;
}

void timeline_free(timeline_t *tl) {
    if (!tl) return;
    for (int i = 0; i < TL_MAX_VEHICLES; i++) free(tl->tracks[i].ring);
    for (uint32_t i = 0; i < tl->kf_count; i++) free(tl->keyframes[i].blob);
    free(tl->keyframes);
    free(tl->events);
    free(tl->rays);
    memset(tl, 0, sizeof(*tl));
}

void timeline_clear(timeline_t *tl) {
    if (!tl) return;
    for (int i = 0; i < TL_MAX_VEHICLES; i++) {
        tl->tracks[i].count = tl->tracks[i].head = 0;
        tl->tracks[i].total = 0;
    }
    for (uint32_t i = 0; i < tl->kf_count; i++) free(tl->keyframes[i].blob);
    tl->kf_count = 0;
    tl->kf_bytes = 0;
    tl->kf_last_ns = INT64_MIN;
    tl->event_count = tl->event_head = 0;
    tl->ray_count = tl->ray_head = 0;
    tl->ray_total = tl->ray_oldest = 0;
    tl->map_cursor = 0;
    tl->map_valid = true;
    tl->have_span = false;
}

size_t timeline_bytes(const timeline_t *tl) {
    if (!tl) return 0;
    size_t total = (size_t)tl->ray_cap * sizeof(tl_ray_t)
                 + (size_t)tl->event_cap * sizeof(tl_event_t)
                 + tl->kf_bytes;
    for (int i = 0; i < TL_MAX_VEHICLES; i++)
        total += (size_t)tl->tracks[i].cap * sizeof(tl_sample_t);
    return total;
}

// ---------------------------------------------------------------- ingest

static void note_time(timeline_t *tl, int64_t t_ns) {
    if (!tl->have_span) {
        tl->start_ns = t_ns;
        tl->head_ns = t_ns;
        tl->have_span = true;
        if (tl->playhead.pinned_to_head) tl->playhead.t_ns = t_ns;
        return;
    }
    if (t_ns > tl->head_ns) tl->head_ns = t_ns;
    if (t_ns < tl->start_ns) tl->start_ns = t_ns;
}

void timeline_add_sample(timeline_t *tl, uint8_t vehicle_id, const tl_sample_t *s) {
    if (!tl || !s) return;
    tl_track_t *tr = &tl->tracks[vehicle_id];
    if (!tr->ring) {
        tr->cap = pow2_at_least(tl->track_bytes / sizeof(tl_sample_t));
        tr->ring = (tl_sample_t *)calloc(tr->cap, sizeof(tl_sample_t));
        if (!tr->ring) { tr->cap = 0; return; }
        tr->active = true;
    }
    tr->ring[tr->head] = *s;
    tr->head = (tr->head + 1) & (tr->cap - 1);
    if (tr->count < tr->cap) tr->count++;
    tr->total++;
    note_time(tl, s->t_ns);
}

void timeline_add_event(timeline_t *tl, const tl_event_t *e) {
    if (!tl || !e || !tl->events) return;
    tl->events[tl->event_head] = *e;
    tl->events[tl->event_head].text[TL_EVENT_TEXT - 1] = '\0';
    tl->event_head = (tl->event_head + 1) & (tl->event_cap - 1);
    if (tl->event_count < tl->event_cap) tl->event_count++;
    note_time(tl, e->t_ns);
}

void timeline_note_drop(timeline_t *tl, int64_t t_ns, uint8_t vehicle_id, uint32_t dropped) {
    if (!tl || dropped == 0) return;
    tl_event_t e;
    memset(&e, 0, sizeof(e));
    e.t_ns = t_ns;
    e.vehicle_id = vehicle_id;
    e.kind = TL_EVENT_MAP_DROP;
    e.severity = 4;   // MAV_SEVERITY_WARNING
    e.code = dropped;
    snprintf(e.text, sizeof(e.text), "map overloaded: %u rays dropped", dropped);
    timeline_add_event(tl, &e);
}

// Drop keyframes whose rays have aged out of the log. A keyframe is only usable
// while every ray recorded after it is still available.
static void drop_stale_keyframes(timeline_t *tl) {
    uint32_t keep = 0;
    for (uint32_t i = 0; i < tl->kf_count; i++) {
        if (tl->keyframes[i].ray_index >= tl->ray_oldest) {
            tl->keyframes[keep++] = tl->keyframes[i];
        } else {
            tl->kf_bytes -= tl->keyframes[i].len;
            free(tl->keyframes[i].blob);
        }
    }
    tl->kf_count = keep;
}

uint64_t timeline_add_ray(timeline_t *tl, const om_ray_t *ray, int64_t t_ns) {
    if (!tl || !ray || !tl->rays) return 0;

    tl_ray_t r;
    r.t_ns = t_ns;
    for (int i = 0; i < 3; i++) {
        r.origin[i] = (float)ray->origin[i];
        r.endpoint[i] = (float)ray->endpoint[i];
    }
    r.vehicle_id = ray->vehicle_id;
    r.hit = ray->hit ? 1 : 0;
    float w = ray->weight;
    if (w < 0.0f) w = 0.0f;
    if (w > 1.0f) w = 1.0f;
    r.weight_q = (uint8_t)lrintf(w * 255.0f);
    float cone_cm = ray->cone_radius_m * 100.0f;
    if (cone_cm < 0.0f) cone_cm = 0.0f;
    if (cone_cm > 65535.0f) cone_cm = 65535.0f;
    r.cone_cm = (uint16_t)lrintf(cone_cm);
    r.flags = 0;

    const uint64_t seq = tl->ray_total;
    tl->rays[tl->ray_head] = r;
    tl->ray_head = (tl->ray_head + 1) & (tl->ray_cap - 1);
    if (tl->ray_count < tl->ray_cap) {
        tl->ray_count++;
    } else {
        tl->ray_oldest++;
        drop_stale_keyframes(tl);
    }
    tl->ray_total++;
    note_time(tl, t_ns);
    return seq;
}

void timeline_mark_ray_skipped(timeline_t *tl, uint64_t seq) {
    if (!tl || !tl->rays) return;
    if (seq < tl->ray_oldest || seq >= tl->ray_total) return;   // already evicted
    const uint32_t start = (tl->ray_head - tl->ray_count) & (tl->ray_cap - 1);
    const uint32_t off = (uint32_t)(seq - tl->ray_oldest);
    tl->rays[(start + off) & (tl->ray_cap - 1)].flags |= TL_RAY_SKIPPED;
}

// ---------------------------------------------------------------- query

// Logical index i (0 == oldest held) into the ring.
static const tl_sample_t *track_at(const tl_track_t *tr, uint32_t i) {
    const uint32_t start = (tr->head - tr->count) & (tr->cap - 1);
    return &tr->ring[(start + i) & (tr->cap - 1)];
}

// Greatest logical index whose timestamp is <= t_ns, or -1.
static int32_t track_search(const tl_track_t *tr, int64_t t_ns) {
    if (tr->count == 0) return -1;
    if (track_at(tr, 0)->t_ns > t_ns) return -1;
    uint32_t lo = 0, hi = tr->count - 1;
    while (lo < hi) {
        const uint32_t mid = lo + (hi - lo + 1) / 2;
        if (track_at(tr, mid)->t_ns <= t_ns) lo = mid; else hi = mid - 1;
    }
    return (int32_t)lo;
}

bool timeline_sample_at(const timeline_t *tl, uint8_t vehicle_id, int64_t t_ns,
                        tl_sample_t *out) {
    if (!tl || !out) return false;
    const tl_track_t *tr = &tl->tracks[vehicle_id];
    if (!tr->ring || tr->count == 0) return false;
    const int32_t i = track_search(tr, t_ns);
    if (i < 0) { *out = *track_at(tr, 0); return true; }
    *out = *track_at(tr, (uint32_t)i);
    return true;
}

static void quat_slerp_short(const float a[4], const float b[4], float t, float out[4]) {
    float dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
    float bb[4] = { b[0], b[1], b[2], b[3] };
    if (dot < 0.0f) { dot = -dot; for (int i = 0; i < 4; i++) bb[i] = -bb[i]; }
    if (dot > 0.9995f) {
        for (int i = 0; i < 4; i++) out[i] = a[i] + (bb[i] - a[i]) * t;
    } else {
        const float theta = acosf(dot);
        const float st = sinf(theta);
        const float wa = sinf((1.0f - t) * theta) / st;
        const float wb = sinf(t * theta) / st;
        for (int i = 0; i < 4; i++) out[i] = a[i] * wa + bb[i] * wb;
    }
    const float n = sqrtf(out[0]*out[0] + out[1]*out[1] + out[2]*out[2] + out[3]*out[3]);
    if (n > 1e-9f) for (int i = 0; i < 4; i++) out[i] /= n;
}

bool timeline_pose_at(const timeline_t *tl, uint8_t vehicle_id, int64_t t_ns,
                      tl_sample_t *out) {
    if (!tl || !out) return false;
    const tl_track_t *tr = &tl->tracks[vehicle_id];
    if (!tr->ring || tr->count == 0) return false;

    const int32_t i = track_search(tr, t_ns);
    if (i < 0) { *out = *track_at(tr, 0); return true; }
    if ((uint32_t)i + 1 >= tr->count) { *out = *track_at(tr, (uint32_t)i); return true; }

    const tl_sample_t *a = track_at(tr, (uint32_t)i);
    const tl_sample_t *b = track_at(tr, (uint32_t)i + 1);
    const int64_t span = b->t_ns - a->t_ns;
    if (span <= 0) { *out = *a; return true; }

    const float u = (float)((double)(t_ns - a->t_ns) / (double)span);
    *out = *a;
    out->t_ns = t_ns;
    for (int k = 0; k < 3; k++) {
        out->enu[k] = a->enu[k] + (b->enu[k] - a->enu[k]) * (double)u;
        out->vel_ned[k] = a->vel_ned[k] + (b->vel_ned[k] - a->vel_ned[k]) * u;
    }
    quat_slerp_short(a->q, b->q, u, out->q);
    return true;
}

int timeline_events_between(const timeline_t *tl, int64_t t0, int64_t t1,
                            const tl_event_t **out, int max_out) {
    if (!tl || !out || max_out <= 0 || tl->event_count == 0) return 0;
    int n = 0;
    const uint32_t start = (tl->event_head - tl->event_count) & (tl->event_cap - 1);
    for (uint32_t k = tl->event_count; k-- > 0 && n < max_out; ) {
        const tl_event_t *e = &tl->events[(start + k) & (tl->event_cap - 1)];
        if (e->t_ns < t0) break;   // events are appended in time order
        if (e->t_ns <= t1) out[n++] = e;
    }
    return n;
}

int64_t timeline_head_ns(const timeline_t *tl) {
    return (tl && tl->have_span) ? tl->head_ns : 0;
}

int64_t timeline_start_ns(const timeline_t *tl) {
    return (tl && tl->have_span) ? tl->start_ns : 0;
}

int64_t timeline_map_history_start_ns(const timeline_t *tl) {
    if (!tl) return 0;
    if (tl->ray_oldest == 0) return timeline_start_ns(tl);
    // The oldest ray still held; anything before it cannot be rebuilt.
    const uint32_t idx = (tl->ray_head - tl->ray_count) & (tl->ray_cap - 1);
    return tl->rays[idx].t_ns;
}

// ---------------------------------------------------------------- playhead

void timeline_pin_live(timeline_t *tl) {
    if (!tl) return;
    const bool was_scrubbed = !tl->playhead.pinned_to_head;
    tl->playhead.pinned_to_head = true;
    tl->playhead.paused = false;
    if (tl->have_span) tl->playhead.t_ns = tl->head_ns;
    // The map is still showing whatever the scrub reconstructed. Invalidate it
    // so the next sync rebuilds forward to the head; without this the map stays
    // frozen in the past while the playhead reads live.
    if (was_scrubbed) tl->map_valid = false;
}

void timeline_unpin(timeline_t *tl) {
    if (tl) tl->playhead.pinned_to_head = false;
}

void timeline_set_playhead(timeline_t *tl, int64_t t_ns) {
    if (!tl) return;
    tl->playhead.pinned_to_head = false;
    if (tl->have_span) {
        if (t_ns < tl->start_ns) t_ns = tl->start_ns;
        if (t_ns > tl->head_ns) t_ns = tl->head_ns;
    }
    tl->playhead.t_ns = t_ns;
}

void timeline_advance(timeline_t *tl, float dt_s) {
    if (!tl || !tl->have_span) return;

    if (tl->playhead.pinned_to_head) {
        tl->playhead.t_ns = tl->head_ns;
        return;
    }
    if (tl->playhead.step_request) {
        tl->playhead.step_request = false;
        // One 60 Hz frame at the current speed; enough to see a single update.
        tl->playhead.t_ns += (int64_t)(1e9 / 60.0 * (double)tl->playhead.speed);
    } else if (!tl->playhead.paused) {
        tl->playhead.t_ns += (int64_t)((double)dt_s * (double)tl->playhead.speed * 1e9);
    }
    if (tl->playhead.t_ns < tl->start_ns) tl->playhead.t_ns = tl->start_ns;
    if (tl->playhead.t_ns >= tl->head_ns) {
        tl->playhead.t_ns = tl->head_ns;
        // Running off the end of recorded data is what "catching up to live"
        // looks like, so re-pin rather than stalling at the boundary.
        tl->playhead.pinned_to_head = true;
    }
}

// ---------------------------------------------------------------- keyframes

bool timeline_take_keyframe(timeline_t *tl, const octomap_t *map, int64_t t_ns) {
    if (!tl || !map) return false;
    const size_t need = octomap_snapshot_size(map);
    if (need == 0) return false;

    // Evict oldest keyframes until the new one fits the byte budget.
    while (tl->kf_count > 0 && tl->kf_bytes + need > tl->kf_bytes_cap) {
        tl->kf_bytes -= tl->keyframes[0].len;
        free(tl->keyframes[0].blob);
        memmove(&tl->keyframes[0], &tl->keyframes[1],
                (size_t)(tl->kf_count - 1) * sizeof(tl_keyframe_t));
        tl->kf_count--;
    }
    if (tl->kf_bytes + need > tl->kf_bytes_cap) return false;

    if (tl->kf_count == tl->kf_cap) {
        const uint32_t cap = tl->kf_cap * 2;
        tl_keyframe_t *k = (tl_keyframe_t *)realloc(tl->keyframes, cap * sizeof(tl_keyframe_t));
        if (!k) return false;
        tl->keyframes = k;
        tl->kf_cap = cap;
    }

    void *blob = malloc(need);
    if (!blob) return false;
    if (octomap_snapshot(map, blob, need) != need) { free(blob); return false; }

    tl->keyframes[tl->kf_count].t_ns = t_ns;
    tl->keyframes[tl->kf_count].ray_index = tl->map_cursor;
    tl->keyframes[tl->kf_count].blob = blob;
    tl->keyframes[tl->kf_count].len = need;
    tl->kf_count++;
    tl->kf_bytes += need;
    tl->kf_last_ns = t_ns;
    return true;
}

void timeline_maybe_keyframe(timeline_t *tl, const octomap_t *map, int64_t t_ns) {
    if (!tl || !map) return;
    if (tl->kf_last_ns != INT64_MIN &&
        t_ns - tl->kf_last_ns < (int64_t)(tl->kf_interval_s * 1e9f)) return;
    timeline_take_keyframe(tl, map, t_ns);
}

// ---------------------------------------------------------------- map sync

static const tl_ray_t *ray_by_lifetime_index(const timeline_t *tl, uint64_t idx) {
    if (idx < tl->ray_oldest || idx >= tl->ray_total) return NULL;
    const uint32_t start = (tl->ray_head - tl->ray_count) & (tl->ray_cap - 1);
    const uint32_t off = (uint32_t)(idx - tl->ray_oldest);
    return &tl->rays[(start + off) & (tl->ray_cap - 1)];
}

static void replay_ray(octomap_t *map, const tl_ray_t *r) {
    om_ray_t ray;
    memset(&ray, 0, sizeof(ray));
    for (int i = 0; i < 3; i++) {
        ray.origin[i] = (double)r->origin[i];
        ray.endpoint[i] = (double)r->endpoint[i];
    }
    ray.hit = r->hit != 0;
    ray.weight = (float)r->weight_q / 255.0f;
    ray.cone_radius_m = (float)r->cone_cm * 0.01f;
    ray.vehicle_id = r->vehicle_id;
    ray.time_ms = (uint32_t)(r->t_ns / 1000000LL);
    octomap_insert_ray(map, &ray);
}

uint32_t timeline_sync_map(timeline_t *tl, octomap_t *map) {
    if (!tl || !map || !tl->rays) return 0;
    const int64_t target = tl->playhead.t_ns;

    // Backward move (or an invalidated map): restore the newest keyframe at or
    // before the target and replay forward from there.
    if (!tl->map_valid || target < tl->map_state_ns) {
        int best = -1;
        for (uint32_t i = 0; i < tl->kf_count; i++)
            if (tl->keyframes[i].t_ns <= target) best = (int)i;

        if (best >= 0 && octomap_restore(map, tl->keyframes[best].blob, tl->keyframes[best].len)) {
            tl->map_cursor = tl->keyframes[best].ray_index;
            tl->map_state_ns = tl->keyframes[best].t_ns;
        } else {
            octomap_clear(map);
            tl->map_cursor = tl->ray_oldest;
            tl->map_state_ns = INT64_MIN;
        }
        tl->map_valid = true;
    }

    if (tl->map_cursor < tl->ray_oldest) tl->map_cursor = tl->ray_oldest;

    uint32_t replayed = 0;
    while (tl->map_cursor < tl->ray_total) {
        const tl_ray_t *r = ray_by_lifetime_index(tl, tl->map_cursor);
        if (!r) break;
        if (r->t_ns > target) break;
        // A ray the queue shed never reached the live map, so replaying it
        // would make the reconstruction denser than what was actually shown.
        if (!(r->flags & TL_RAY_SKIPPED)) {
            replay_ray(map, r);
            replayed++;
        }
        tl->map_cursor++;
    }
    tl->map_state_ns = target;
    return replayed;
}
