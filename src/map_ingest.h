#ifndef MAP_INGEST_H
#define MAP_INGEST_H

// Bounded ray queue between decoding and the map.
//
// Full free-space carving is not free: 72 sectors x 20 Hz x 16 vehicles is a
// lot of tree walking. When the producer outruns the map we drop the *oldest*
// pending rays and count them, because falling quietly behind is worse than
// admitting the loss -- the HUD shows the number, so an operator can tell the
// difference between "nothing there" and "we stopped looking".

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "octomap.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t queue_capacity;    // 0 -> 65536 rays
    uint32_t budget_per_drain;  // 0 -> 20000 rays per drain call
    float    prune_interval_s;  // 0 -> 5.0
    float    prune_pressure;    // 0 -> 0.80 of byte_cap before an early prune
} map_ingest_config_t;

typedef struct {
    uint64_t enqueued;
    uint64_t inserted;
    uint64_t dropped;           // shed at the queue because the map fell behind
    uint64_t rejected;          // decoded but unusable (below min range, no pose)
    uint32_t queue_depth;
    uint32_t queue_capacity;
    float    rays_per_s;        // smoothed insertion rate
    float    drops_per_s;       // smoothed drop rate
    uint32_t last_prune_reclaimed;
    uint32_t prune_count;
} map_ingest_stats_t;

// Each queued ray carries the lifetime index it has in the timeline ray log,
// so the map's progress can be reported in the log's own terms rather than
// assumed. Assuming was how keyframes came to claim rays still sitting here.
typedef struct {
    om_ray_t ray;
    uint64_t seq;
} mi_entry_t;

typedef struct {
    mi_entry_t *ring;
    uint32_t  cap;
    uint32_t  head;     // next slot to write
    uint32_t  tail;     // next slot to read
    uint32_t  count;

    uint32_t  budget;
    float     prune_interval_s;
    float     prune_pressure;
    // The last prune pass reclaimed nothing, so pressure alone will not
    // trigger another -- only the interval will. See maintain().
    bool      prune_barren;
    float     since_prune_s;

    // Rate smoothing, updated once per tick.
    float     accum_s;
    uint64_t  accum_inserted;
    uint64_t  accum_dropped;

    // Lifetime index of the newest ray actually inserted into the map, plus a
    // small ring of indices shed under overload so the caller can mark them in
    // the history. Without this, live and replay disagree about what the map
    // was ever built from.
    uint64_t  last_inserted_seq;
    bool      have_inserted;
    uint64_t  dropped_seq[64];
    uint32_t  dropped_seq_count;

    map_ingest_stats_t stats;
} map_ingest_t;

int  map_ingest_init(map_ingest_t *mi, const map_ingest_config_t *cfg);
void map_ingest_free(map_ingest_t *mi);
void map_ingest_reset(map_ingest_t *mi);

// Enqueue a ray, tagged with its lifetime index in the timeline ray log.
// Returns false when an older ray had to be shed to make room.
bool map_ingest_push(map_ingest_t *mi, const om_ray_t *ray, uint64_t seq);

// Enqueue a batch that shares an origin, with consecutive indices from `seq0`.
// Returns the number of rays shed.
uint32_t map_ingest_push_batch(map_ingest_t *mi, const om_ray_t *rays, int count,
                               uint64_t seq0);

// Drop everything pending without inserting it. Used while the playhead is
// scrubbed: those rays belong in the history, but not in a view of the past.
// They are not counted as overload drops.
uint32_t map_ingest_discard_all(map_ingest_t *mi);

// Take the indices shed since the last call. Returns the count written.
uint32_t map_ingest_take_dropped(map_ingest_t *mi, uint64_t *out, uint32_t max_out);

// Count an observation that decoded but carried nothing usable.
void map_ingest_note_rejected(map_ingest_t *mi);

// Insert up to the per-drain budget into the map. Returns rays inserted.
uint32_t map_ingest_drain(map_ingest_t *mi, octomap_t *map, float dt);

// Drain everything regardless of budget. Used by replay seeks and by the
// offline checkers, where wall-clock pacing is not a constraint.
uint32_t map_ingest_drain_all(map_ingest_t *mi, octomap_t *map);

const map_ingest_stats_t *map_ingest_stats(const map_ingest_t *mi);

// True when rays are being shed right now -- what the HUD annunciator watches.
bool map_ingest_overloaded(const map_ingest_t *mi);

#ifdef __cplusplus
}
#endif

#endif
