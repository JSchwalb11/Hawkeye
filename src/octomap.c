#include "octomap.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define OM_NODES_PER_BLOCK 8
#define OM_INITIAL_NODES   (1u << 16)
#define OM_CHUNK_INITIAL   1024

// ---------------------------------------------------------------- config

void octomap_config_defaults(octomap_config_t *cfg) {
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->root_size_m   = 4096.0;
    cfg->max_depth     = 14;      // 0.25 m leaves
    cfg->coarse_depth  = 11;      // 2 m far-field carving
    cfg->chunk_depth   = 9;       // 8 m render chunks
    cfg->byte_cap      = (size_t)256 * 1024 * 1024;
    cfg->refine_dist_m = 1.5;
    cfg->skip_near_m   = 1.0;
}

double octomap_cell_size(const octomap_t *m, int depth) {
    return m->root_size / (double)(1u << depth);
}

int octomap_depth_for_size(const octomap_t *m, double size_m) {
    if (size_m <= 0.0) return m->max_depth;
    int d = (int)ceil(log2(m->root_size / size_m));
    if (d < 0) d = 0;
    if (d > m->max_depth) d = m->max_depth;
    return d;
}

size_t octomap_bytes(const octomap_t *m) {
    return (size_t)m->node_cap * sizeof(om_node_t)
         + (size_t)m->chunk_cap * sizeof(om_chunk_slot_t);
}

// ---------------------------------------------------------------- chunks

static uint64_t chunk_hash(int64_t key) {
    uint64_t h = (uint64_t)key;
    h ^= h >> 33; h *= 0xff51afd7ed558ccdULL;
    h ^= h >> 33; h *= 0xc4ceb9fe1a85ec53ULL;
    h ^= h >> 33;
    return h;
}

static bool chunk_table_alloc(octomap_t *m, uint32_t cap) {
    om_chunk_slot_t *t = (om_chunk_slot_t *)calloc(cap, sizeof(om_chunk_slot_t));
    if (!t) return false;
    free(m->chunks);
    m->chunks = t;
    m->chunk_cap = cap;
    m->chunk_count = 0;
    m->dirty_count = 0;
    return true;
}

static void chunk_mark_dirty(octomap_t *m, int64_t key);

static bool chunk_table_grow(octomap_t *m) {
    const uint32_t old_cap = m->chunk_cap;
    om_chunk_slot_t *old = m->chunks;
    // Detach before reallocating: chunk_table_alloc frees whatever is attached,
    // and the old table is still needed for the rehash below.
    m->chunks = NULL;
    if (!chunk_table_alloc(m, old_cap ? old_cap * 2 : OM_CHUNK_INITIAL)) {
        m->chunks = old;
        return false;
    }
    for (uint32_t i = 0; i < old_cap; i++) {
        if (!old[i].occupied) continue;
        // Reinsert; dirtiness is carried across so nothing loses its pending
        // re-extraction just because the table resized.
        const int64_t key = old[i].key;
        const uint32_t was_dirty = old[i].dirty;
        uint64_t h = chunk_hash(key) & (m->chunk_cap - 1);
        while (m->chunks[h].occupied) h = (h + 1) & (m->chunk_cap - 1);
        m->chunks[h].occupied = 1;
        m->chunks[h].key = key;
        m->chunks[h].dirty = was_dirty;
        m->chunk_count++;
        if (was_dirty) m->dirty_count++;
    }
    free(old);
    return true;
}

static void chunk_mark_dirty(octomap_t *m, int64_t key) {
    if (!m->chunks && !chunk_table_alloc(m, OM_CHUNK_INITIAL)) return;
    if ((m->chunk_count + 1) * 4 >= m->chunk_cap * 3) {
        if (!chunk_table_grow(m)) return;
    }
    uint64_t h = chunk_hash(key) & (m->chunk_cap - 1);
    while (m->chunks[h].occupied) {
        if (m->chunks[h].key == key) {
            if (!m->chunks[h].dirty) { m->chunks[h].dirty = 1; m->dirty_count++; }
            return;
        }
        h = (h + 1) & (m->chunk_cap - 1);
    }
    m->chunks[h].occupied = 1;
    m->chunks[h].key = key;
    m->chunks[h].dirty = 1;
    m->chunk_count++;
    m->dirty_count++;
}

static int64_t pack_chunk(int32_t ix, int32_t iy, int32_t iz) {
    // 21 bits each, biased. Enough for 2^21 chunks per axis at any sane depth.
    const int64_t bx = (int64_t)(ix + (1 << 20)) & 0x1FFFFF;
    const int64_t by = (int64_t)(iy + (1 << 20)) & 0x1FFFFF;
    const int64_t bz = (int64_t)(iz + (1 << 20)) & 0x1FFFFF;
    return bx | (by << 21) | (bz << 42);
}

static void unpack_chunk(int64_t key, int32_t *ix, int32_t *iy, int32_t *iz) {
    *ix = (int32_t)(key & 0x1FFFFF) - (1 << 20);
    *iy = (int32_t)((key >> 21) & 0x1FFFFF) - (1 << 20);
    *iz = (int32_t)((key >> 42) & 0x1FFFFF) - (1 << 20);
}

int64_t octomap_chunk_key(const octomap_t *m, double x, double y, double z) {
    const double s = octomap_cell_size(m, m->chunk_depth);
    return pack_chunk((int32_t)floor(x / s), (int32_t)floor(y / s), (int32_t)floor(z / s));
}

void octomap_chunk_center(const octomap_t *m, int64_t key, double out[3], double *out_size) {
    const double s = octomap_cell_size(m, m->chunk_depth);
    int32_t ix, iy, iz;
    unpack_chunk(key, &ix, &iy, &iz);
    out[0] = ((double)ix + 0.5) * s;
    out[1] = ((double)iy + 0.5) * s;
    out[2] = ((double)iz + 0.5) * s;
    if (out_size) *out_size = s;
}

bool octomap_chunk_is_dirty(const octomap_t *m, int64_t key) {
    if (m->all_dirty) return true;
    if (!m->chunks) return false;
    uint64_t h = chunk_hash(key) & (m->chunk_cap - 1);
    for (uint32_t probe = 0; probe < m->chunk_cap; probe++) {
        if (!m->chunks[h].occupied) return false;
        if (m->chunks[h].key == key) return m->chunks[h].dirty != 0;
        h = (h + 1) & (m->chunk_cap - 1);
    }
    return false;
}

void octomap_chunk_clear_dirty(octomap_t *m, int64_t key) {
    if (!m->chunks) return;
    uint64_t h = chunk_hash(key) & (m->chunk_cap - 1);
    for (uint32_t probe = 0; probe < m->chunk_cap; probe++) {
        if (!m->chunks[h].occupied) return;
        if (m->chunks[h].key == key) {
            if (m->chunks[h].dirty) { m->chunks[h].dirty = 0; m->dirty_count--; }
            return;
        }
        h = (h + 1) & (m->chunk_cap - 1);
    }
}

void octomap_chunks_clear_all_dirty(octomap_t *m) {
    m->all_dirty = false;
    if (!m->chunks) return;
    for (uint32_t i = 0; i < m->chunk_cap; i++) m->chunks[i].dirty = 0;
    m->dirty_count = 0;
}

bool octomap_chunk_next(const octomap_t *m, uint32_t *cursor, int64_t *out_key) {
    if (!m->chunks) return false;
    for (uint32_t i = *cursor; i < m->chunk_cap; i++) {
        if (m->chunks[i].occupied) {
            *out_key = m->chunks[i].key;
            *cursor = i + 1;
            return true;
        }
    }
    *cursor = m->chunk_cap;
    return false;
}

// ---------------------------------------------------------------- pool

static bool pool_reserve(octomap_t *m, uint32_t needed) {
    if (needed <= m->node_cap) return true;
    uint32_t cap = m->node_cap ? m->node_cap : OM_INITIAL_NODES;
    while (cap < needed) {
        if (cap > (UINT32_MAX / 2)) return false;
        cap *= 2;
    }
    const size_t want = (size_t)cap * sizeof(om_node_t)
                      + (size_t)m->chunk_cap * sizeof(om_chunk_slot_t);
    if (m->byte_cap && want > m->byte_cap) return false;

    om_node_t *n = (om_node_t *)realloc(m->nodes, (size_t)cap * sizeof(om_node_t));
    if (!n) return false;
    memset(n + m->node_cap, 0, (size_t)(cap - m->node_cap) * sizeof(om_node_t));
    m->nodes = n;
    m->node_cap = cap;
    return true;
}

// Allocate a block of 8 children, reusing a pruned block when one is free.
static uint32_t block_alloc(octomap_t *m) {
    if (m->free_head) {
        const uint32_t blk = m->free_head;
        m->free_head = m->nodes[blk].children;
        m->free_blocks--;
        memset(&m->nodes[blk], 0, OM_NODES_PER_BLOCK * sizeof(om_node_t));
        return blk;
    }
    if (!pool_reserve(m, m->node_count + OM_NODES_PER_BLOCK)) {
        m->stats.alloc_refusals++;
        return 0;
    }
    const uint32_t blk = m->node_count;
    m->node_count += OM_NODES_PER_BLOCK;
    if (m->node_count > m->stats.peak_nodes) m->stats.peak_nodes = m->node_count;
    const size_t bytes = octomap_bytes(m);
    if (bytes > m->stats.peak_bytes) m->stats.peak_bytes = bytes;
    return blk;
}

static void block_free(octomap_t *m, uint32_t blk) {
    memset(&m->nodes[blk], 0, OM_NODES_PER_BLOCK * sizeof(om_node_t));
    m->nodes[blk].children = m->free_head;
    m->free_head = blk;
    m->free_blocks++;
}

// ---------------------------------------------------------------- lifecycle

int octomap_init(octomap_t *m, const octomap_config_t *cfg) {
    if (!m) return -1;
    octomap_config_t d;
    octomap_config_defaults(&d);
    if (cfg) {
        if (cfg->root_size_m   > 0.0) d.root_size_m   = cfg->root_size_m;
        if (cfg->max_depth     > 0)   d.max_depth     = cfg->max_depth;
        if (cfg->coarse_depth  > 0)   d.coarse_depth  = cfg->coarse_depth;
        if (cfg->chunk_depth   > 0)   d.chunk_depth   = cfg->chunk_depth;
        if (cfg->byte_cap      > 0)   d.byte_cap      = cfg->byte_cap;
        if (cfg->refine_dist_m > 0.0) d.refine_dist_m = cfg->refine_dist_m;
        if (cfg->skip_near_m  >= 0.0) d.skip_near_m   = cfg->skip_near_m;
    }
    if (d.max_depth > 20) d.max_depth = 20;
    if (d.coarse_depth > d.max_depth) d.coarse_depth = d.max_depth;
    if (d.chunk_depth > d.max_depth) d.chunk_depth = d.max_depth;

    memset(m, 0, sizeof(*m));
    m->root_size     = d.root_size_m;
    m->root_half     = d.root_size_m * 0.5;
    m->max_depth     = d.max_depth;
    m->coarse_depth  = d.coarse_depth;
    m->chunk_depth   = d.chunk_depth;
    m->refine_dist_m = d.refine_dist_m;
    m->skip_near_m   = d.skip_near_m;
    m->byte_cap      = d.byte_cap;

    // A hit is worth about twice a miss, which is the usual occupancy-grid
    // ratio: surfaces should be easy to see and hard to invent.
    m->lo_hit          = 17;
    m->lo_miss         = -8;
    m->occ_threshold   = OM_OCC_THRESHOLD;
    m->free_threshold  = OM_FREE_THRESHOLD;
    m->prune_tolerance = 6;
    m->contested_pct   = 30;
    m->contested_min_votes = 4;

    if (!pool_reserve(m, OM_INITIAL_NODES)) return -1;
    m->node_count = 1;  // root
    if (!chunk_table_alloc(m, OM_CHUNK_INITIAL)) { free(m->nodes); m->nodes = NULL; return -1; }
    return 0;
}

void octomap_free(octomap_t *m) {
    if (!m) return;
    free(m->nodes);
    free(m->chunks);
    m->nodes = NULL;
    m->chunks = NULL;
    m->node_cap = m->node_count = 0;
    m->chunk_cap = m->chunk_count = 0;
}

void octomap_clear(octomap_t *m) {
    if (!m || !m->nodes) return;
    memset(m->nodes, 0, (size_t)m->node_cap * sizeof(om_node_t));
    m->node_count = 1;
    m->free_head = 0;
    m->free_blocks = 0;
    if (m->chunks) memset(m->chunks, 0, (size_t)m->chunk_cap * sizeof(om_chunk_slot_t));
    m->chunk_count = 0;
    m->dirty_count = 0;
    m->all_dirty = true;
    // Counters that describe the map's contents reset; the lifetime throughput
    // counters do not, because the HUD reports them as session totals.
    m->stats.peak_nodes = 1;
    m->stats.peak_bytes = octomap_bytes(m);
}

// ---------------------------------------------------------------- update

static void node_apply(octomap_t *m, uint32_t idx, int delta, uint8_t vehicle_id,
                       uint32_t time_ms) {
    om_node_t *n = &m->nodes[idx];
    const uint32_t bit = om_vehicle_bit(vehicle_id);

    // Divergence is a fleet signal, not a temporal one. Only evidence from a
    // vehicle other than the ones already on record can contest a cell, so a
    // single vehicle watching an obstacle get removed clears it cleanly instead
    // of painting the whole area contested.
    const uint32_t others = n->observers & ~bit;
    if (others != 0 && (n->log_odds >= m->occ_threshold || n->log_odds <= m->free_threshold)) {
        const bool node_occupied = n->log_odds >= m->occ_threshold;
        const bool evidence_occupied = delta > 0;
        if (node_occupied == evidence_occupied) {
            if (n->agree < 255) n->agree++;
        } else {
            if (n->disagree < 255) n->disagree++;
        }
        if (n->agree == 255 && n->disagree == 255) { n->agree = 128; n->disagree = 128; }
    }

    int lo = (int)n->log_odds + delta;
    if (lo >  OM_LO_CLAMP) lo =  OM_LO_CLAMP;
    if (lo < -OM_LO_CLAMP) lo = -OM_LO_CLAMP;
    n->log_odds = (int8_t)lo;
    n->observers |= bit;
    n->last_seen_ms = time_ms;
    m->stats.node_updates++;
}

// Split a leaf, seeding every child with the parent's evidence so the split is
// information-preserving. Returns false when the memory cap refuses it.
static bool node_subdivide(octomap_t *m, uint32_t idx) {
    if (m->nodes[idx].children) return true;
    const uint32_t blk = block_alloc(m);
    if (!blk) return false;
    const om_node_t parent = m->nodes[idx];
    for (int i = 0; i < OM_NODES_PER_BLOCK; i++) {
        om_node_t *c = &m->nodes[blk + i];
        c->children = 0;
        c->log_odds = parent.log_odds;
        c->observers = parent.observers;
        c->last_seen_ms = parent.last_seen_ms;
        c->agree = parent.agree;
        c->disagree = parent.disagree;
        c->flags = parent.flags;
    }
    m->nodes[idx].children = blk;
    m->stats.subdivisions++;
    return true;
}

static int child_index(double px, double py, double pz, double cx, double cy, double cz) {
    return (px >= cx ? 1 : 0) | (py >= cy ? 2 : 0) | (pz >= cz ? 4 : 0);
}

static void child_center(int i, double cx, double cy, double cz, double quarter,
                         double out[3]) {
    out[0] = cx + ((i & 1) ? quarter : -quarter);
    out[1] = cy + ((i & 2) ? quarter : -quarter);
    out[2] = cz + ((i & 4) ? quarter : -quarter);
}

// Dirty-marking. A leaf coarser than the chunk depth spans many chunks; rather
// than enumerating them we flip the global flag and let the renderer re-extract.
static void mark_dirty_at(octomap_t *m, double x, double y, double z, int depth) {
    if (depth >= m->chunk_depth) chunk_mark_dirty(m, octomap_chunk_key(m, x, y, z));
    else m->all_dirty = true;
}

// Apply evidence at a point, descending to `target_depth`. Occupied evidence
// always resolves to the target; free evidence only splits confident occupancy.
static void apply_point(octomap_t *m, const double p[3], int target_depth,
                        int delta, uint8_t vehicle_id, uint32_t time_ms) {
    if (fabs(p[0]) >= m->root_half || fabs(p[1]) >= m->root_half ||
        fabs(p[2]) >= m->root_half) return;

    uint32_t idx = 0;
    int depth = 0;
    double cx = 0.0, cy = 0.0, cz = 0.0;
    double half = m->root_half;

    for (;;) {
        if (depth >= target_depth) {
            // Reaching the target depth is not the same as reaching a leaf. The
            // carve pass runs before the hit and may already have subdivided
            // this node, and every reader descends past interior nodes to a
            // true leaf -- so stopping here would write the evidence somewhere
            // nothing ever looks.
            if (!m->nodes[idx].children) break;
        } else if (!m->nodes[idx].children) {
            const bool must_split = (delta > 0) || (m->nodes[idx].log_odds >= m->occ_threshold);
            if (!must_split) break;
            if (!node_subdivide(m, idx)) break;
        }
        const double quarter = half * 0.5;
        const int ci = child_index(p[0], p[1], p[2], cx, cy, cz);
        idx = m->nodes[idx].children + (uint32_t)ci;
        cx += (ci & 1) ? quarter : -quarter;
        cy += (ci & 2) ? quarter : -quarter;
        cz += (ci & 4) ? quarter : -quarter;
        half = quarter;
        depth++;
    }

    node_apply(m, idx, delta, vehicle_id, time_ms);
    mark_dirty_at(m, p[0], p[1], p[2], depth);
}

// ---------------------------------------------------------------- carving

typedef struct {
    double o[3];
    double d[3];     // unit direction
    double inv[3];
    double len;      // metres from o to the endpoint
} carve_ray_t;

static bool slab_clip(const carve_ray_t *r, const double lo[3], const double hi[3],
                      double *t0, double *t1) {
    double tmin = *t0, tmax = *t1;
    for (int a = 0; a < 3; a++) {
        if (r->d[a] == 0.0) {
            if (r->o[a] < lo[a] || r->o[a] > hi[a]) return false;
        } else {
            double ta = (lo[a] - r->o[a]) * r->inv[a];
            double tb = (hi[a] - r->o[a]) * r->inv[a];
            if (ta > tb) { const double tmp = ta; ta = tb; tb = tmp; }
            if (ta > tmin) tmin = ta;
            if (tb < tmax) tmax = tb;
            if (tmin > tmax) return false;
        }
    }
    *t0 = tmin; *t1 = tmax;
    return true;
}

// Depth this stretch of the ray deserves: coarse in the far field, and near the
// endpoint as fine as the *sensor* can justify. A 25-degree sonar cannot place a
// surface to a quarter of a metre, so refining its neighbourhood that finely
// would invent detail -- the cone's own footprint is the resolution limit, and
// that is what makes a sonar's cells visibly coarser than a laser's.
static int carve_depth_for(const octomap_t *m, double dist_to_end, int fine_depth) {
    return (dist_to_end > m->refine_dist_m) ? m->coarse_depth : fine_depth;
}

// `marked` says an ancestor at or above the chunk depth has already flagged the
// chunk this subtree lives in, so deep recursion costs one hash probe per ray
// segment rather than one per node update.
static void carve_rec(octomap_t *m, uint32_t idx, int depth,
                      double cx, double cy, double cz, double half,
                      const carve_ray_t *r, double t_enter, double t_exit,
                      int delta, uint8_t vehicle_id, uint32_t time_ms, bool marked,
                      int fine_depth) {
    int want = carve_depth_for(m, r->len - t_exit, fine_depth);

    // Where the ray contradicts confident occupancy, resolve it properly even
    // in the far field. A grazing ray that clips the corner of a 2 m cell must
    // not be allowed to clear a wall it never actually passed through.
    if (m->nodes[idx].log_odds >= m->occ_threshold) want = fine_depth;

    if (!marked && depth >= m->chunk_depth) {
        chunk_mark_dirty(m, octomap_chunk_key(m, cx, cy, cz));
        marked = true;
    }

    if (!m->nodes[idx].children) {
        if (depth >= want) {
            node_apply(m, idx, delta, vehicle_id, time_ms);
            if (!marked) m->all_dirty = true;
            return;
        }
        // Carving must reach `want` before it applies anything. Stopping at
        // whatever leaf happened to be there would mark a 16 m cube free on the
        // strength of one pencil-thin ray -- free space would balloon into
        // volumes nothing ever looked at.
        //
        // The adaptivity is in `want` itself (coarse in the far field, full
        // resolution near the endpoint) and in pruning, which collapses the
        // uniform siblings this creates.
        if (!node_subdivide(m, idx)) {
            // Refused at the memory cap: apply here rather than lose the ray,
            // and let the refusal counter say what happened.
            node_apply(m, idx, delta, vehicle_id, time_ms);
            if (!marked) m->all_dirty = true;
            return;
        }
    }

    const double quarter = half * 0.5;
    const uint32_t base = m->nodes[idx].children;
    for (int i = 0; i < OM_NODES_PER_BLOCK; i++) {
        double cc[3];
        child_center(i, cx, cy, cz, quarter, cc);
        const double lo[3] = { cc[0] - quarter, cc[1] - quarter, cc[2] - quarter };
        const double hi[3] = { cc[0] + quarter, cc[1] + quarter, cc[2] + quarter };
        double a = t_enter, b = t_exit;
        if (!slab_clip(r, lo, hi, &a, &b)) continue;
        if (b <= a) continue;
        carve_rec(m, base + (uint32_t)i, depth + 1, cc[0], cc[1], cc[2], quarter,
                  r, a, b, delta, vehicle_id, time_ms, marked, fine_depth);
    }
}

static int scale_delta(int base, float weight) {
    if (weight <= 0.0f) return 0;
    if (weight > 1.0f) weight = 1.0f;
    int d = (int)lrintf((float)base * weight);
    if (d == 0) d = (base > 0) ? 1 : -1;   // a weak return still counts for something
    return d;
}

void octomap_insert_ray(octomap_t *m, const om_ray_t *ray) {
    if (!m || !m->nodes || !ray) return;

    double d[3] = {
        ray->endpoint[0] - ray->origin[0],
        ray->endpoint[1] - ray->origin[1],
        ray->endpoint[2] - ray->origin[2],
    };
    double len = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (!(len > 1e-6)) return;
    d[0] /= len; d[1] /= len; d[2] /= len;

    m->stats.rays_inserted++;
    if (ray->hit) m->stats.hits_inserted++;
    else          m->stats.misses_inserted++;

    // The cell the hit will own has to be decided before carving, because the
    // carve must stop short of exactly that cell. Widen the endpoint with the
    // sensor cone: a 25-degree sonar at 10 m is not a laser, and drawing it as
    // one invents detail the sensor never had.
    int hit_depth = m->max_depth;
    if (ray->hit && ray->cone_radius_m > 0.0f) {
        const double want = (double)ray->cone_radius_m * 2.0;
        hit_depth = octomap_depth_for_size(m, want);
        if (hit_depth > m->max_depth) hit_depth = m->max_depth;
        if (hit_depth < m->coarse_depth) hit_depth = m->coarse_depth;
    }

    // Skip the first stretch: that volume is the airframe, not the world.
    double t_start = m->skip_near_m;
    // Stop short by a full hit cell, not half a leaf. Reaching into the cell
    // the hit is about to claim debits every hit by a miss, which is how a
    // surface observed once ends up never crossing the occupied threshold.
    const double hit_cell = octomap_cell_size(m, hit_depth);
    double t_end = len - (ray->hit ? hit_cell : 0.0);
    if (t_end > t_start) {
        carve_ray_t r;
        memcpy(r.o, ray->origin, sizeof(r.o));
        memcpy(r.d, d, sizeof(r.d));
        for (int a = 0; a < 3; a++) r.inv[a] = (d[a] != 0.0) ? 1.0 / d[a] : 0.0;
        r.len = len;

        const double lo[3] = { -m->root_half, -m->root_half, -m->root_half };
        const double hi[3] = {  m->root_half,  m->root_half,  m->root_half };
        double a = t_start, b = t_end;
        if (slab_clip(&r, lo, hi, &a, &b) && b > a) {
            carve_rec(m, 0, 0, 0.0, 0.0, 0.0, m->root_half, &r, a, b,
                      scale_delta(m->lo_miss, ray->weight), ray->vehicle_id,
                      ray->time_ms, false, hit_depth);
        }
    }

    if (!ray->hit) return;   // a max-range reading is "no return", not a wall

    apply_point(m, ray->endpoint, hit_depth,
                scale_delta(m->lo_hit, ray->weight), ray->vehicle_id, ray->time_ms);
}

void octomap_insert_batch(octomap_t *m, const om_ray_t *rays, int count) {
    for (int i = 0; i < count; i++) octomap_insert_ray(m, &rays[i]);
}

// ---------------------------------------------------------------- query

const om_node_t *octomap_lookup(const octomap_t *m, double x, double y, double z,
                                int *out_depth, double out_center[3], double *out_size) {
    if (!m || !m->nodes) return NULL;
    if (fabs(x) >= m->root_half || fabs(y) >= m->root_half || fabs(z) >= m->root_half)
        return NULL;

    uint32_t idx = 0;
    int depth = 0;
    double cx = 0.0, cy = 0.0, cz = 0.0, half = m->root_half;
    while (m->nodes[idx].children) {
        const double quarter = half * 0.5;
        const int ci = child_index(x, y, z, cx, cy, cz);
        idx = m->nodes[idx].children + (uint32_t)ci;
        cx += (ci & 1) ? quarter : -quarter;
        cy += (ci & 2) ? quarter : -quarter;
        cz += (ci & 4) ? quarter : -quarter;
        half = quarter;
        depth++;
    }
    if (out_depth) *out_depth = depth;
    if (out_center) { out_center[0] = cx; out_center[1] = cy; out_center[2] = cz; }
    if (out_size) *out_size = half * 2.0;
    return &m->nodes[idx];
}

om_state_t octomap_query(const octomap_t *m, double x, double y, double z) {
    const om_node_t *n = octomap_lookup(m, x, y, z, NULL, NULL, NULL);
    if (!n) return OM_UNKNOWN;
    return om_node_state(m, n);
}

// ---------------------------------------------------------------- pruning

static bool prune_rec(octomap_t *m, uint32_t idx, uint32_t *reclaimed) {
    if (!m->nodes[idx].children) return true;   // already a leaf

    const uint32_t base = m->nodes[idx].children;
    bool all_leaves = true;
    for (int i = 0; i < OM_NODES_PER_BLOCK; i++) {
        if (!prune_rec(m, base + (uint32_t)i, reclaimed)) all_leaves = false;
    }
    if (!all_leaves) return false;

    int lo_min = 127, lo_max = -128;
    uint32_t observers = 0, last_seen = 0;
    uint8_t agree = 0, disagree = 0;
    int lo_sum = 0;
    for (int i = 0; i < OM_NODES_PER_BLOCK; i++) {
        const om_node_t *c = &m->nodes[base + (uint32_t)i];
        if (c->log_odds < lo_min) lo_min = c->log_odds;
        if (c->log_odds > lo_max) lo_max = c->log_odds;
        lo_sum += c->log_odds;
        observers |= c->observers;
        if (c->last_seen_ms > last_seen) last_seen = c->last_seen_ms;
        if (c->agree > agree) agree = c->agree;
        if (c->disagree > disagree) disagree = c->disagree;
    }
    if (lo_max - lo_min > (int)m->prune_tolerance) return false;

    // Never collapse a contested cell: the whole point of flagging divergence
    // is that it survives long enough to be looked at.
    const int votes = (int)agree + (int)disagree;
    if (votes >= m->contested_min_votes &&
        (int)disagree * 100 >= votes * (int)m->contested_pct) return false;

    om_node_t *p = &m->nodes[idx];
    p->log_odds = (int8_t)(lo_sum / OM_NODES_PER_BLOCK);
    p->observers = observers;
    p->last_seen_ms = last_seen;
    p->agree = agree;
    p->disagree = disagree;
    p->children = 0;
    block_free(m, base);
    m->stats.prunes++;
    (*reclaimed)++;
    return true;
}

uint32_t octomap_prune(octomap_t *m) {
    if (!m || !m->nodes) return 0;
    uint32_t reclaimed = 0;
    prune_rec(m, 0, &reclaimed);
    if (reclaimed) m->all_dirty = true;
    return reclaimed;
}

// ---------------------------------------------------------------- iteration

static void iterate_rec(const octomap_t *m, uint32_t idx, int depth,
                        double cx, double cy, double cz, double half,
                        om_leaf_fn fn, void *user) {
    if (m->nodes[idx].children) {
        const double quarter = half * 0.5;
        const uint32_t base = m->nodes[idx].children;
        for (int i = 0; i < OM_NODES_PER_BLOCK; i++) {
            double cc[3];
            child_center(i, cx, cy, cz, quarter, cc);
            iterate_rec(m, base + (uint32_t)i, depth + 1, cc[0], cc[1], cc[2], quarter, fn, user);
        }
        return;
    }
    om_leaf_t leaf;
    leaf.node = &m->nodes[idx];
    leaf.center[0] = cx; leaf.center[1] = cy; leaf.center[2] = cz;
    leaf.size = half * 2.0;
    leaf.depth = depth;
    leaf.state = om_node_state(m, &m->nodes[idx]);
    fn(&leaf, user);
}

void octomap_iterate(const octomap_t *m, om_leaf_fn fn, void *user) {
    if (!m || !m->nodes || !fn) return;
    iterate_rec(m, 0, 0, 0.0, 0.0, 0.0, m->root_half, fn, user);
}

void octomap_iterate_chunk(const octomap_t *m, int64_t chunk_key,
                           om_leaf_fn fn, void *user) {
    if (!m || !m->nodes || !fn) return;
    double center[3], size;
    octomap_chunk_center(m, chunk_key, center, &size);

    // Descend to the chunk cell, then hand the subtree to the leaf walker. A
    // leaf coarser than the chunk depth is reported once, clipped to the chunk.
    uint32_t idx = 0;
    int depth = 0;
    double cx = 0.0, cy = 0.0, cz = 0.0, half = m->root_half;
    if (fabs(center[0]) >= m->root_half || fabs(center[1]) >= m->root_half ||
        fabs(center[2]) >= m->root_half) return;

    while (depth < m->chunk_depth && m->nodes[idx].children) {
        const double quarter = half * 0.5;
        const int ci = child_index(center[0], center[1], center[2], cx, cy, cz);
        idx = m->nodes[idx].children + (uint32_t)ci;
        cx += (ci & 1) ? quarter : -quarter;
        cy += (ci & 2) ? quarter : -quarter;
        cz += (ci & 4) ? quarter : -quarter;
        half = quarter;
        depth++;
    }
    iterate_rec(m, idx, depth, cx, cy, cz, half, fn, user);
}

// Collapse a subtree into one representative cell: occupied wins over free,
// free wins over unknown, and observers and contest counts accumulate. Used
// only by the renderer's LOD, so a little pessimism costs nothing.
typedef struct { int lo_max, lo_min; uint32_t observers; uint8_t agree, disagree; uint32_t seen; } om_agg_t;

static void aggregate_rec(const octomap_t *m, uint32_t idx, om_agg_t *a) {
    const om_node_t *n = &m->nodes[idx];
    if (!n->children) {
        if (n->log_odds > a->lo_max) a->lo_max = n->log_odds;
        if (n->log_odds < a->lo_min) a->lo_min = n->log_odds;
        a->observers |= n->observers;
        if (n->agree > a->agree) a->agree = n->agree;
        if (n->disagree > a->disagree) a->disagree = n->disagree;
        if (n->last_seen_ms > a->seen) a->seen = n->last_seen_ms;
        return;
    }
    for (int i = 0; i < OM_NODES_PER_BLOCK; i++)
        aggregate_rec(m, n->children + (uint32_t)i, a);
}

typedef struct {
    const octomap_t *m;
    int        max_depth;
    om_leaf_fn fn;
    void      *user;
    om_node_t  scratch;
} lod_ctx_t;

static void lod_rec(lod_ctx_t *c, uint32_t idx, int depth,
                    double cx, double cy, double cz, double half) {
    const octomap_t *m = c->m;
    const om_node_t *n = &m->nodes[idx];

    if (n->children && depth < c->max_depth) {
        const double quarter = half * 0.5;
        for (int i = 0; i < OM_NODES_PER_BLOCK; i++) {
            double cc[3];
            child_center(i, cx, cy, cz, quarter, cc);
            lod_rec(c, n->children + (uint32_t)i, depth + 1, cc[0], cc[1], cc[2], quarter);
        }
        return;
    }

    om_leaf_t leaf;
    if (n->children) {
        om_agg_t a = { -128, 127, 0, 0, 0, 0 };
        aggregate_rec(m, idx, &a);
        c->scratch.children = 0;
        c->scratch.log_odds = (int8_t)(a.lo_max >= m->occ_threshold ? a.lo_max : a.lo_min);
        c->scratch.observers = a.observers;
        c->scratch.agree = a.agree;
        c->scratch.disagree = a.disagree;
        c->scratch.last_seen_ms = a.seen;
        leaf.node = &c->scratch;
    } else {
        leaf.node = n;
    }
    leaf.center[0] = cx; leaf.center[1] = cy; leaf.center[2] = cz;
    leaf.size = half * 2.0;
    leaf.depth = depth;
    leaf.state = om_node_state(m, leaf.node);
    c->fn(&leaf, c->user);
}

void octomap_iterate_chunk_lod(const octomap_t *m, int64_t chunk_key, int max_depth,
                               om_leaf_fn fn, void *user) {
    if (!m || !m->nodes || !fn) return;
    if (max_depth < m->chunk_depth) max_depth = m->chunk_depth;
    if (max_depth > m->max_depth) max_depth = m->max_depth;

    double center[3], size;
    octomap_chunk_center(m, chunk_key, center, &size);
    if (fabs(center[0]) >= m->root_half || fabs(center[1]) >= m->root_half ||
        fabs(center[2]) >= m->root_half) return;

    uint32_t idx = 0;
    int depth = 0;
    double cx = 0.0, cy = 0.0, cz = 0.0, half = m->root_half;
    while (depth < m->chunk_depth && m->nodes[idx].children) {
        const double quarter = half * 0.5;
        const int ci = child_index(center[0], center[1], center[2], cx, cy, cz);
        idx = m->nodes[idx].children + (uint32_t)ci;
        cx += (ci & 1) ? quarter : -quarter;
        cy += (ci & 2) ? quarter : -quarter;
        cz += (ci & 4) ? quarter : -quarter;
        half = quarter;
        depth++;
    }

    lod_ctx_t c;
    memset(&c, 0, sizeof(c));
    c.m = m;
    c.max_depth = max_depth;
    c.fn = fn;
    c.user = user;
    lod_rec(&c, idx, depth, cx, cy, cz, half);
}

// ---------------------------------------------------------------- snapshots

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t node_count;
    uint32_t free_head;
    uint32_t free_blocks;
    uint32_t reserved;
} om_snapshot_header_t;

#define OM_SNAPSHOT_MAGIC   0x4F4D5031u   /* "OMP1" */
#define OM_SNAPSHOT_VERSION 1u

size_t octomap_snapshot_size(const octomap_t *m) {
    if (!m) return 0;
    return sizeof(om_snapshot_header_t) + (size_t)m->node_count * sizeof(om_node_t);
}

size_t octomap_snapshot(const octomap_t *m, void *buf, size_t buf_len) {
    const size_t need = octomap_snapshot_size(m);
    if (!buf || buf_len < need) return 0;
    om_snapshot_header_t h = {
        OM_SNAPSHOT_MAGIC, OM_SNAPSHOT_VERSION,
        m->node_count, m->free_head, m->free_blocks, 0
    };
    memcpy(buf, &h, sizeof(h));
    memcpy((char *)buf + sizeof(h), m->nodes, (size_t)m->node_count * sizeof(om_node_t));
    return need;
}

// Walk a restored tree and re-register every chunk that holds content. The
// snapshot carries only the node array, and the renderer discovers work solely
// by enumerating chunks -- so without this a restored map draws as nothing.
static void reindex_chunks(octomap_t *m, uint32_t idx, int depth,
                           double cx, double cy, double cz, double half) {
    if (depth >= m->chunk_depth) {
        // Anything below here belongs to this chunk; register it if the subtree
        // holds evidence at all.
        om_agg_t a = { -128, 127, 0, 0, 0, 0 };
        aggregate_rec(m, idx, &a);
        if (a.lo_max >= m->occ_threshold || a.lo_min <= m->free_threshold)
            chunk_mark_dirty(m, octomap_chunk_key(m, cx, cy, cz));
        return;
    }
    if (!m->nodes[idx].children) {
        // A leaf coarser than the chunk depth spans many chunks; the renderer
        // handles that through the global flag rather than enumerating them.
        if (m->nodes[idx].log_odds >= m->occ_threshold ||
            m->nodes[idx].log_odds <= m->free_threshold) m->all_dirty = true;
        return;
    }
    const double quarter = half * 0.5;
    const uint32_t base = m->nodes[idx].children;
    for (int i = 0; i < OM_NODES_PER_BLOCK; i++) {
        double cc[3];
        child_center(i, cx, cy, cz, quarter, cc);
        reindex_chunks(m, base + (uint32_t)i, depth + 1, cc[0], cc[1], cc[2], quarter);
    }
}

bool octomap_restore(octomap_t *m, const void *buf, size_t len) {
    if (!m || !buf || len < sizeof(om_snapshot_header_t)) return false;
    om_snapshot_header_t h;
    memcpy(&h, buf, sizeof(h));
    if (h.magic != OM_SNAPSHOT_MAGIC || h.version != OM_SNAPSHOT_VERSION) return false;
    if (len < sizeof(h) + (size_t)h.node_count * sizeof(om_node_t)) return false;
    if (!pool_reserve(m, h.node_count ? h.node_count : 1)) return false;

    memcpy(m->nodes, (const char *)buf + sizeof(h), (size_t)h.node_count * sizeof(om_node_t));
    if (h.node_count < m->node_cap)
        memset(m->nodes + h.node_count, 0,
               (size_t)(m->node_cap - h.node_count) * sizeof(om_node_t));
    m->node_count = h.node_count ? h.node_count : 1;
    m->free_head = h.free_head;
    m->free_blocks = h.free_blocks;
    if (m->chunks) memset(m->chunks, 0, (size_t)m->chunk_cap * sizeof(om_chunk_slot_t));
    m->chunk_count = 0;
    m->dirty_count = 0;
    m->all_dirty = true;
    reindex_chunks(m, 0, 0, 0.0, 0.0, 0.0, m->root_half);
    return true;
}
