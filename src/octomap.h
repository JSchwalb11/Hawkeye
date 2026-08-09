#ifndef OCTOMAP_H
#define OCTOMAP_H

// Fleet-shared adaptive octree occupancy map.
//
// One merged map for the whole fleet: every vehicle's rays go into the same
// tree, each cell remembers which vehicles contributed, and cells where two
// vehicles disagree are flagged rather than silently averaged.
//
// Structure notes worth knowing before editing:
//
//  * Nodes live in a flat pool addressed by uint32 indices, allocated eight at
//    a time. No malloc'd pointers, so the tree survives a realloc and stays
//    cache-friendly. Index 0 is the root, so `children == 0` means "leaf".
//
//  * Subdivision is evidence-driven. Occupied evidence resolves down to the
//    target depth because surfaces are the detail we care about; free evidence
//    only splits a node that currently holds confident occupancy. Uniform free
//    space therefore stays coarse, which is what keeps a long flight bounded.
//
//  * Pruning collapses eight sibling leaves back into their parent once they
//    agree. It is the half that people forget, and it is what makes memory
//    plateau instead of climb.
//
//  * Carving is hierarchical: the recursion stops at a coarse depth while it is
//    far from the ray endpoint and refines to full resolution only for the last
//    couple of metres. Same free/unknown/occupied semantics, far less work.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Log-odds are stored in an int8. One unit is OM_LO_SCALE of natural log-odds.
//
// The clamp matters more than it looks. A cell that is allowed to saturate at
// near-certainty needs a very long stream of contrary evidence to change its
// mind, and an obstacle that is removed then never clears. Bounding confidence
// at roughly +/-3.5 log-odds keeps the map able to be wrong and then corrected,
// which is the whole point of carving.
#define OM_LO_SCALE       0.05f
#define OM_LO_CLAMP       70     // ~+/-3.5 log-odds
#define OM_OCC_THRESHOLD   14    // ~0.7 probability
#define OM_FREE_THRESHOLD (-14)

// Observers are a 32-bit mask. Fleets larger than 32 fold with `% 32`; the
// contribution highlight folds identically, so the two always agree.
#define OM_MAX_TRACKED_VEHICLES 32

typedef struct {
    uint32_t children;      // index of the first of 8 children; 0 == leaf
    uint32_t observers;     // bitmask of contributing vehicles
    uint32_t last_seen_ms;  // session milliseconds of the last update
    int8_t   log_odds;
    uint8_t  agree;         // cross-vehicle corroboration, saturating
    uint8_t  disagree;      // cross-vehicle contradiction, saturating
    // Bit 0 marks a cell whose last applied update was a range return. It is
    // what gives `hit_grace_ms` something to measure from -- see node_apply.
    // Bit 1 marks a cell that has been told where inside itself the surface is.
    uint8_t  flags;
    // Where in this cell the returns actually landed, in units of
    // OM_SURF_SCALE per cell edge, relative to the centre. A cell says
    // "occupied" but a range return knows more than that: it knows *where*.
    // Discarding the remainder rounds every surface to the nearest cell centre
    // and puts a floor of about a third of a cell under the map's accuracy that
    // no amount of evidence can lift. See octomap_surface_point.
    int8_t   surf[3];
} om_node_t;

#define OM_FLAG_FRESH_HIT   0x01
#define OM_FLAG_HAS_SURFACE 0x02

// `surf` spans the cell: -127..127 maps to -0.5..+0.5 of an edge. The quantum
// is a cell/254, three orders below the leaf, so the storage is never the
// limit -- the sensor is.
#define OM_SURF_SCALE 254.0

typedef enum {
    OM_UNKNOWN = 0,
    OM_FREE,
    OM_OCCUPIED,
} om_state_t;

typedef struct {
    uint64_t rays_inserted;
    uint64_t hits_inserted;
    uint64_t misses_inserted;      // max-range returns: carve, mark nothing
    uint64_t node_updates;
    uint64_t subdivisions;
    uint64_t prunes;               // child blocks collapsed
    uint64_t alloc_refusals;       // subdivision refused at the memory cap
    uint32_t peak_nodes;
    size_t   peak_bytes;
} om_stats_t;

typedef struct {
    int64_t  key;        // packed chunk coordinate
    uint32_t occupied;   // slot in use
    uint32_t dirty;
} om_chunk_slot_t;

typedef struct {
    om_node_t *nodes;
    uint32_t   node_count;   // high-water index into the pool
    uint32_t   node_cap;
    uint32_t   free_head;    // head of the free list of 8-node blocks (0 == empty)
    uint32_t   free_blocks;

    double     root_size;    // edge length of the root cube, metres
    double     root_half;
    int        max_depth;    // leaf edge = root_size / 2^max_depth
    int        coarse_depth; // depth used for far free-space carving
    int        chunk_depth;  // depth at which the renderer chunks the tree

    double     refine_dist_m;   // carve at full resolution within this of the endpoint
    double     skip_near_m;     // ignore the first stretch of every ray (the airframe)

    int8_t     lo_hit;
    int8_t     lo_miss;
    int8_t     occ_threshold;
    int8_t     free_threshold;
    uint8_t    prune_tolerance;      // max sibling log-odds spread that still collapses
    // How long a range return shields its cell from free evidence. A grazing
    // pass is a sub-second burst; a removed obstacle keeps producing misses for
    // as long as anyone looks. Time is what separates them -- a ray count
    // cannot, because both arrive in bulk.
    uint32_t   hit_grace_ms;

    uint8_t    contested_pct;        // disagree share, in percent, that marks a cell contested
    uint8_t    contested_min_votes;  // minimum agree+disagree before a cell can be contested

    size_t     byte_cap;     // hard ceiling on the node pool

    om_chunk_slot_t *chunks;
    uint32_t   chunk_cap;    // power of two
    uint32_t   chunk_count;
    uint32_t   dirty_count;
    bool       all_dirty;

    om_stats_t stats;
} octomap_t;

typedef struct {
    double  root_size_m;   // 0 -> 4096
    int     max_depth;     // 0 -> 14 (0.25 m leaves at the default root)
    int     coarse_depth;  // 0 -> depth whose cell is nearest 2 m
    int     chunk_depth;   // 0 -> depth whose cell is nearest 8 m
    size_t  byte_cap;      // 0 -> 256 MiB
    double  refine_dist_m; // 0 -> 1.5
    double  skip_near_m;   // 0 -> 1.0
    int     hit_grace_ms;  // 0 -> 1000; negative disables the shield
} octomap_config_t;

void octomap_config_defaults(octomap_config_t *cfg);

// Returns 0 on success.
int  octomap_init(octomap_t *m, const octomap_config_t *cfg);
void octomap_free(octomap_t *m);

// Drop all evidence but keep the configuration. Used when scrubbing backwards
// past the oldest map keyframe.
void octomap_clear(octomap_t *m);

double octomap_cell_size(const octomap_t *m, int depth);
int    octomap_depth_for_size(const octomap_t *m, double size_m);
size_t octomap_bytes(const octomap_t *m);

// Bytes actually holding tree, ignoring pool the free list is sitting on.
//
// `octomap_bytes` reports capacity, and the pool never shrinks -- so it can
// only ever rise, and anything that compares it against the cap to decide
// "the map is getting full" latches on permanently the first time the pool
// grows. This is the figure that falls again when pruning reclaims blocks.
size_t octomap_live_bytes(const octomap_t *m);

// --- Insertion ---------------------------------------------------------

typedef struct {
    double  origin[3];      // ray start, session ENU metres
    double  endpoint[3];    // ray end
    bool    hit;            // false == "no return"; carve, mark nothing occupied
    float   weight;         // evidence scale in [0,1]; weak returns move less
    float   cone_radius_m;  // endpoint half-width from the sensor FOV
    uint8_t vehicle_id;
    uint32_t time_ms;       // session milliseconds
} om_ray_t;

void octomap_insert_ray(octomap_t *m, const om_ray_t *ray);

// A batch shares one origin and one tree-descent prefix, which is how an
// OBSTACLE_DISTANCE message's 72 sectors get inserted for the price of one.
void octomap_insert_batch(octomap_t *m, const om_ray_t *rays, int count);

// --- Query -------------------------------------------------------------

om_state_t octomap_query(const octomap_t *m, double x, double y, double z);
const om_node_t *octomap_lookup(const octomap_t *m, double x, double y, double z,
                                int *out_depth, double out_center[3], double *out_size);

static inline bool om_node_contested(const octomap_t *m, const om_node_t *n) {
    const int votes = (int)n->agree + (int)n->disagree;
    if (votes < m->contested_min_votes) return false;
    return (int)n->disagree * 100 >= votes * (int)m->contested_pct;
}

static inline om_state_t om_node_state(const octomap_t *m, const om_node_t *n) {
    if (n->log_odds >= m->occ_threshold) return OM_OCCUPIED;
    if (n->log_odds <= m->free_threshold) return OM_FREE;
    return OM_UNKNOWN;
}

static inline uint32_t om_vehicle_bit(uint8_t vehicle_id) {
    return 1u << (vehicle_id % OM_MAX_TRACKED_VEHICLES);
}

// The map's best estimate of where the surface inside this cell is, rather than
// where the cell is. Falls back to the centre for a cell no return ever landed
// in -- carved-free and unknown cells have no surface to report, and neither
// does an occupied cell that only ever inherited its evidence from a prune.
//
// Anything scoring or drawing a surface should ask for this instead of using
// the cell centre. The difference is invisible where the cell is smaller than
// the sensor's own error and worth roughly a factor of two where it is not.
static inline void om_node_surface(const om_node_t *n, const double center[3],
                                   double size, double out[3]) {
    if (!(n->flags & OM_FLAG_HAS_SURFACE)) {
        out[0] = center[0]; out[1] = center[1]; out[2] = center[2];
        return;
    }
    const double s = size / OM_SURF_SCALE;
    out[0] = center[0] + (double)n->surf[0] * s;
    out[1] = center[1] + (double)n->surf[1] * s;
    out[2] = center[2] + (double)n->surf[2] * s;
}

// --- Maintenance -------------------------------------------------------

// Collapse agreeing sibling leaves. Returns the number of blocks reclaimed.
uint32_t octomap_prune(octomap_t *m);

// --- Iteration ---------------------------------------------------------

typedef struct {
    const om_node_t *node;
    double center[3];
    double size;
    int    depth;
    om_state_t state;
} om_leaf_t;

typedef void (*om_leaf_fn)(const om_leaf_t *leaf, void *user);

void octomap_iterate(const octomap_t *m, om_leaf_fn fn, void *user);

// Visit only the leaves inside one chunk-depth cell.
void octomap_iterate_chunk(const octomap_t *m, int64_t chunk_key,
                           om_leaf_fn fn, void *user);

// As above, but stop descending at `max_depth` and report the node there as a
// single aggregate cell. This is the renderer's level of detail: octree depth
// is the LOD, which is the second reason adaptive was the right structure.
void octomap_iterate_chunk_lod(const octomap_t *m, int64_t chunk_key, int max_depth,
                               om_leaf_fn fn, void *user);

// --- Chunk bookkeeping for the renderer --------------------------------

int64_t octomap_chunk_key(const octomap_t *m, double x, double y, double z);
void    octomap_chunk_center(const octomap_t *m, int64_t key, double out[3], double *out_size);

// Axis-aligned bounds of everything the map holds, to chunk resolution. Reads
// the chunk index rather than walking the tree, so it is cheap enough to call
// every frame -- which is what a camera that frames the map needs. Returns
// false when the map is empty.
bool    octomap_content_bounds(const octomap_t *m, double lo[3], double hi[3]);
bool    octomap_chunk_is_dirty(const octomap_t *m, int64_t key);
void    octomap_chunk_clear_dirty(octomap_t *m, int64_t key);
void    octomap_chunks_clear_all_dirty(octomap_t *m);

// Enumerate live chunk keys. `cursor` starts at 0; returns false when done.
bool    octomap_chunk_next(const octomap_t *m, uint32_t *cursor, int64_t *out_key);

// --- Snapshots (map keyframes for scrubbing) ---------------------------

size_t octomap_snapshot_size(const octomap_t *m);
size_t octomap_snapshot(const octomap_t *m, void *buf, size_t buf_len);
bool   octomap_restore(octomap_t *m, const void *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif
