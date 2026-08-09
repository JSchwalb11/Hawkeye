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

    mi->ring = (mi_entry_t *)calloc(cap, sizeof(mi_entry_t));
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

bool map_ingest_push(map_ingest_t *mi, const om_ray_t *ray, uint64_t seq) {
    if (!mi || !mi->ring || !ray) return false;
    bool ok = true;
    if (mi->count == mi->cap) {
        // Shed the oldest: a stale ray is worth less than the one arriving.
        // Remember which one, so the history can record that the map never
        // saw it rather than quietly disagreeing with itself on replay.
        const uint64_t lost = mi->ring[mi->tail].seq;
        if (mi->dropped_seq_count < (uint32_t)(sizeof(mi->dropped_seq) / sizeof(mi->dropped_seq[0])))
            mi->dropped_seq[mi->dropped_seq_count++] = lost;
        mi->tail = (mi->tail + 1) & (mi->cap - 1);
        mi->count--;
        mi->stats.dropped++;
        ok = false;
    }
    mi->ring[mi->head].ray = *ray;
    mi->ring[mi->head].seq = seq;
    mi->head = (mi->head + 1) & (mi->cap - 1);
    mi->count++;
    mi->stats.enqueued++;
    mi->stats.queue_depth = mi->count;
    return ok;
}

uint32_t map_ingest_push_batch(map_ingest_t *mi, const om_ray_t *rays, int count,
                               uint64_t seq0) {
    uint32_t shed = 0;
    for (int i = 0; i < count; i++)
        if (!map_ingest_push(mi, &rays[i], seq0 + (uint64_t)i)) shed++;
    return shed;
}

uint32_t map_ingest_discard_all(map_ingest_t *mi) {
    if (!mi || !mi->ring) return 0;
    const uint32_t n = mi->count;
    mi->tail = mi->head;
    mi->count = 0;
    mi->stats.queue_depth = 0;
    return n;
}

uint32_t map_ingest_take_dropped(map_ingest_t *mi, uint64_t *out, uint32_t max_out) {
    if (!mi || !out) return 0;
    uint32_t n = mi->dropped_seq_count;
    if (n > max_out) n = max_out;
    for (uint32_t i = 0; i < n; i++) out[i] = mi->dropped_seq[i];
    mi->dropped_seq_count = 0;
    return n;
}

void map_ingest_note_rejected(map_ingest_t *mi) {
    if (mi) mi->stats.rejected++;
}

static void maintain(map_ingest_t *mi, octomap_t *map, float dt) {
    mi->since_prune_s += dt;

    // Pressure is measured against the tree, not against the pool.
    //
    // `octomap_bytes` reports capacity, and the pool never shrinks -- so a map
    // that once grew past the pressure fraction stayed above it for the rest of
    // the session no matter how much pruning reclaimed, and a full-tree prune
    // then ran on *every* drain. Measured on statue-solo with a 16 MiB cap:
    // 1741 passes against 24, and sustained throughput fell from 114k rays/s to
    // 7.6k. `octomap_live_bytes` discounts the free list, so a productive prune
    // actually relieves the pressure that triggered it.
    const bool pressured = map->byte_cap > 0 &&
        (double)octomap_live_bytes(map) >
        (double)map->byte_cap * (double)mi->prune_pressure;

    // Even so, a map can be genuinely full with nothing left to collapse: at
    // the cap, subdivision is refused rather than granted, so the tree stops
    // changing shape and every pass finds the same unprunable nodes. A pass
    // that reclaimed nothing is evidence that the next one will too, so drop
    // back to the periodic schedule and let the interval decide when the tree
    // has had time to change. That bounds the worst case at one walk per
    // interval instead of one per drain.
    const bool due = mi->since_prune_s >= mi->prune_interval_s;
    if (!due && (!pressured || mi->prune_barren)) return;

    const uint32_t reclaimed = octomap_prune(map);
    mi->stats.last_prune_reclaimed = reclaimed;
    mi->stats.prune_count++;
    mi->since_prune_s = 0.0f;
    mi->prune_barren = (reclaimed == 0);
}

static uint32_t drain_n(map_ingest_t *mi, octomap_t *map, uint32_t budget) {
    uint32_t n = 0;
    while (n < budget && mi->count > 0) {
        octomap_insert_ray(map, &mi->ring[mi->tail].ray);
        mi->last_inserted_seq = mi->ring[mi->tail].seq;
        mi->have_inserted = true;
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
