#include "map_ingest.h"

#include <stdlib.h>
#include <string.h>

#define MI_DEFAULT_QUEUE   65536u
#define MI_DEFAULT_BUDGET  20000u
#define MI_RATE_WINDOW_S   0.5f

int map_ingest_init(map_ingest_t *mi, const map_ingest_config_t *cfg) {
    if (!mi) return -1;
    memset(mi, 0, sizeof(*mi));

    uint32_t cap = (cfg && cfg->queue_capacity) ? cfg->queue_capacity : MI_DEFAULT_QUEUE;
    // Round up to a power of two so the ring wraps with a mask.
    uint32_t p = 1;
    while (p < cap && p < (1u << 30)) p <<= 1;
    cap = p;

    mi->ring = (om_ray_t *)calloc(cap, sizeof(om_ray_t));
    if (!mi->ring) return -1;
    mi->cap = cap;
    mi->budget = (cfg && cfg->budget_per_drain) ? cfg->budget_per_drain : MI_DEFAULT_BUDGET;
    mi->prune_interval_s = (cfg && cfg->prune_interval_s > 0.0f) ? cfg->prune_interval_s : 5.0f;
    mi->prune_pressure = (cfg && cfg->prune_pressure > 0.0f) ? cfg->prune_pressure : 0.80f;
    mi->stats.queue_capacity = cap;
    return 0;
}

void map_ingest_free(map_ingest_t *mi) {
    if (!mi) return;
    free(mi->ring);
    mi->ring = NULL;
    mi->cap = 0;
}

void map_ingest_reset(map_ingest_t *mi) {
    if (!mi) return;
    mi->head = mi->tail = mi->count = 0;
    mi->stats.queue_depth = 0;
    mi->since_prune_s = 0.0f;
}

bool map_ingest_push(map_ingest_t *mi, const om_ray_t *ray) {
    if (!mi || !mi->ring || !ray) return false;
    bool ok = true;
    if (mi->count == mi->cap) {
        // Shed the oldest: a stale ray is worth less than the one arriving.
        mi->tail = (mi->tail + 1) & (mi->cap - 1);
        mi->count--;
        mi->stats.dropped++;
        ok = false;
    }
    mi->ring[mi->head] = *ray;
    mi->head = (mi->head + 1) & (mi->cap - 1);
    mi->count++;
    mi->stats.enqueued++;
    mi->stats.queue_depth = mi->count;
    return ok;
}

uint32_t map_ingest_push_batch(map_ingest_t *mi, const om_ray_t *rays, int count) {
    uint32_t shed = 0;
    for (int i = 0; i < count; i++)
        if (!map_ingest_push(mi, &rays[i])) shed++;
    return shed;
}

void map_ingest_note_rejected(map_ingest_t *mi) {
    if (mi) mi->stats.rejected++;
}

static void maintain(map_ingest_t *mi, octomap_t *map, float dt) {
    mi->since_prune_s += dt;

    const size_t bytes = octomap_bytes(map);
    const bool pressured = map->byte_cap > 0 &&
        (double)bytes > (double)map->byte_cap * (double)mi->prune_pressure;

    if (pressured || mi->since_prune_s >= mi->prune_interval_s) {
        mi->stats.last_prune_reclaimed = octomap_prune(map);
        mi->stats.prune_count++;
        mi->since_prune_s = 0.0f;
    }
}

static uint32_t drain_n(map_ingest_t *mi, octomap_t *map, uint32_t budget) {
    uint32_t n = 0;
    while (n < budget && mi->count > 0) {
        octomap_insert_ray(map, &mi->ring[mi->tail]);
        mi->tail = (mi->tail + 1) & (mi->cap - 1);
        mi->count--;
        n++;
    }
    mi->stats.inserted += n;
    mi->stats.queue_depth = mi->count;
    return n;
}

static void update_rates(map_ingest_t *mi, float dt, uint32_t inserted) {
    mi->accum_s += dt;
    mi->accum_inserted += inserted;
    if (mi->accum_s >= MI_RATE_WINDOW_S) {
        const float inv = 1.0f / mi->accum_s;
        const float rays = (float)mi->accum_inserted * inv;
        const float drops = (float)(mi->stats.dropped - mi->accum_dropped) * inv;
        // Light smoothing so the HUD number is readable rather than twitchy.
        mi->stats.rays_per_s = mi->stats.rays_per_s * 0.5f + rays * 0.5f;
        mi->stats.drops_per_s = mi->stats.drops_per_s * 0.5f + drops * 0.5f;
        mi->accum_s = 0.0f;
        mi->accum_inserted = 0;
        mi->accum_dropped = mi->stats.dropped;
    }
}

uint32_t map_ingest_drain(map_ingest_t *mi, octomap_t *map, float dt) {
    if (!mi || !mi->ring || !map) return 0;
    const uint32_t n = drain_n(mi, map, mi->budget);
    maintain(mi, map, dt);
    update_rates(mi, dt, n);
    return n;
}

uint32_t map_ingest_drain_all(map_ingest_t *mi, octomap_t *map) {
    if (!mi || !mi->ring || !map) return 0;
    uint32_t total = 0;
    while (mi->count > 0) total += drain_n(mi, map, mi->cap);
    maintain(mi, map, mi->prune_interval_s);
    return total;
}

const map_ingest_stats_t *map_ingest_stats(const map_ingest_t *mi) {
    return mi ? &mi->stats : NULL;
}

bool map_ingest_overloaded(const map_ingest_t *mi) {
    if (!mi) return false;
    return mi->stats.drops_per_s > 0.5f || mi->count * 4 >= mi->cap * 3;
}
