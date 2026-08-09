#include "map_render.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "asset_path.h"
#include "fleet_frame.h"
#include "raymath.h"
#include "rlgl.h"

#define MR_INITIAL_CHUNKS 512
#define MR_CELL_SHRINK    0.92f   // a hair of gap so cell edges stay legible

static const char *const k_mode_names[MAP_DRAW_MODE_COUNT] = {
    "occupancy", "coverage", "divergence", "contribution",
};

const char *map_draw_mode_name(map_draw_mode_t m) {
    if (m < 0 || m >= MAP_DRAW_MODE_COUNT) return "?";
    return k_mode_names[m];
}

// Bucket meanings are shared by every mode; only the palette changes.
enum {
    B_FREE = 0,       // carved space
    B_OCC_WEAK,       // occupied, low confidence
    B_OCC_STRONG,     // occupied, high confidence
    B_CONTESTED,      // fleet disagrees here
    B_FOCUS,          // focused vehicle only
    B_SHARED,         // focused vehicle and others
};

// ---------------------------------------------------------------- chunks

static uint64_t key_hash(int64_t key) {
    uint64_t h = (uint64_t)key;
    h ^= h >> 33; h *= 0xff51afd7ed558ccdULL;
    h ^= h >> 33; h *= 0xc4ceb9fe1a85ec53ULL;
    h ^= h >> 33;
    return h;
}

static void chunk_release(map_chunk_t *c) {
    for (int b = 0; b < MAP_RENDER_BUCKETS; b++) {
        free(c->xf[b]);
        c->xf[b] = NULL;
        c->cap[b] = c->count[b] = 0;
    }
}

static bool table_alloc(map_render_t *r, uint32_t cap) {
    map_chunk_t *t = (map_chunk_t *)calloc(cap, sizeof(map_chunk_t));
    if (!t) return false;
    r->chunks = t;
    r->chunk_cap = cap;
    r->chunk_count = 0;
    return true;
}

static bool table_grow(map_render_t *r) {
    const uint32_t old_cap = r->chunk_cap;
    map_chunk_t *old = r->chunks;
    if (!table_alloc(r, old_cap ? old_cap * 2 : MR_INITIAL_CHUNKS)) {
        r->chunks = old;
        r->chunk_cap = old_cap;
        return false;
    }
    for (uint32_t i = 0; i < old_cap; i++) {
        if (!old[i].used) continue;
        uint64_t h = key_hash(old[i].key) & (r->chunk_cap - 1);
        while (r->chunks[h].used) h = (h + 1) & (r->chunk_cap - 1);
        r->chunks[h] = old[i];
        r->chunk_count++;
    }
    free(old);
    return true;
}

static map_chunk_t *chunk_find(map_render_t *r, int64_t key, bool create) {
    if (!r->chunks && !table_alloc(r, MR_INITIAL_CHUNKS)) return NULL;
    if (create && (r->chunk_count + 1) * 4 >= r->chunk_cap * 3 && !table_grow(r)) return NULL;

    uint64_t h = key_hash(key) & (r->chunk_cap - 1);
    for (uint32_t probe = 0; probe < r->chunk_cap; probe++) {
        if (!r->chunks[h].used) {
            if (!create) return NULL;
            r->chunks[h].used = true;
            r->chunks[h].key = key;
            r->chunks[h].lod_depth = -1;
            r->chunk_count++;
            return &r->chunks[h];
        }
        if (r->chunks[h].key == key) return &r->chunks[h];
        h = (h + 1) & (r->chunk_cap - 1);
    }
    return NULL;
}

static bool bucket_push(map_chunk_t *c, int bucket, Matrix m) {
    if (c->count[bucket] == c->cap[bucket]) {
        const int cap = c->cap[bucket] ? c->cap[bucket] * 2 : 64;
        Matrix *x = (Matrix *)realloc(c->xf[bucket], (size_t)cap * sizeof(Matrix));
        if (!x) return false;
        c->xf[bucket] = x;
        c->cap[bucket] = cap;
    }
    c->xf[bucket][c->count[bucket]++] = m;
    return true;
}

// ---------------------------------------------------------------- extraction

typedef struct {
    map_render_t *r;
    map_chunk_t  *chunk;
    uint32_t      focus_mask;
    const octomap_t *map;

    // Carved cells go through this grid instead of straight into the bucket.
    // NULL wherever meshing does not apply -- a mode that draws no carved
    // space, or a chunk too deep to rasterise. See "free meshing" below.
    uint8_t *grid;
    int      gdim;
    double   gmin[3];     // ENU corner of the chunk, low side on every axis
    double   gcell;
} extract_ctx_t;

// ---------------------------------------------------------------- free meshing
//
// Pruning has already merged whatever agreed cube-wise, and what is left is
// lace: measured on statue-fleet only 30.1% of carved cells are enclosed on all
// six faces, so dropping the interior thinned the veil by 28% and no sharper
// face test does much better.
//
// Greedy meshing merges the runs the octree cannot, because a box need not be a
// cube and need not sit on a power-of-two boundary. The volume is unchanged --
// this is a re-tiling, not an approximation.
//
// Merging cannot cross a chunk, and that is the ceiling on what it can buy: a
// run is at most one chunk long, eight cells for the shipping configuration.
// Even so it drew statue-fleet's veil with 109,075 boxes where the same carved
// cells cost 220,247 cubes.

// A grid this size is 256 KiB. Deeper chunks fall back to a cube per leaf
// rather than allocating a grid whose side doubles per level.
#define MR_MESH_LEVELS_MAX 6
#define MR_MESH_DIM_MAX    (1 << MR_MESH_LEVELS_MAX)

// A merged box needs a wider gap than the cells it stands in for, and holding
// the gap *area* constant is not enough. Shrinking a box by MR_CELL_SHRINK
// leaves exactly the seam area its cells had -- 8% of the span either way --
// but concentrated at the two ends, and the surface behind the veil was
// showing through the seams. Eight small gaps sample what is behind them in
// eight places; one large gap samples it in one. Measured against the
// free-hidden reference, as the share of surface pixels that still read as
// surface:
//
//     a cube per carved cell, 0.92 (what this replaces)   72.2%
//     merged boxes, 0.92 -- the same seam area            67.7%
//     merged boxes, 0.84                                  70.8%
//     merged boxes, 0.78                                  74.7%
//
// The gap is not free: it paints carved space it has evidence for. Counting
// every pixel the veil repaints, 0.78 covers 78,273 against the 87,388 a cube
// per cell covered, so 10% of the veil is traded for the legibility above.
#define MR_BOX_SHRINK 0.78

static void mesh_mark(extract_ctx_t *x, const om_leaf_t *leaf) {
    const int N = x->gdim;
    int lo[3], hi[3];
    int span = (int)llround(leaf->size / x->gcell);
    if (span < 1) span = 1;
    for (int a = 0; a < 3; a++) {
        const double corner = leaf->center[a] - leaf->size * 0.5;
        lo[a] = (int)llround((corner - x->gmin[a]) / x->gcell);
        hi[a] = lo[a] + span - 1;
        // A leaf coarser than the chunk overhangs it -- the LOD walk reports
        // such a node whole rather than clipped to the chunk asking for it --
        // so the range is trimmed here rather than trusted.
        if (lo[a] < 0) lo[a] = 0;
        if (hi[a] >= N) hi[a] = N - 1;
        if (lo[a] > hi[a]) return;
    }
    for (int k = lo[2]; k <= hi[2]; k++)
        for (int j = lo[1]; j <= hi[1]; j++)
            memset(&x->grid[(k * N + j) * N + lo[0]], 1, (size_t)(hi[0] - lo[0] + 1));
}

static bool mesh_row_set(const uint8_t *g, int N, int i0, int i1, int j, int k) {
    for (int i = i0; i <= i1; i++) if (!g[(k * N + j) * N + i]) return false;
    return true;
}

// Grow a box along x, then y, then z, taking each axis as far as it goes before
// moving to the next. Axis order biases which shape wins where several tile the
// same run, and no order is better than another for a volume this irregular.
static void mesh_flush(extract_ctx_t *x) {
    const int N = x->gdim;
    uint8_t *g = x->grid;

    for (int k = 0; k < N; k++)
    for (int j = 0; j < N; j++)
    for (int i = 0; i < N; i++) {
        if (!g[(k * N + j) * N + i]) continue;

        int i1 = i;
        while (i1 + 1 < N && g[(k * N + j) * N + i1 + 1]) i1++;
        int j1 = j;
        while (j1 + 1 < N && mesh_row_set(g, N, i, i1, j1 + 1, k)) j1++;
        int k1 = k;
        while (k1 + 1 < N) {
            bool ok = true;
            for (int jj = j; jj <= j1 && ok; jj++) ok = mesh_row_set(g, N, i, i1, jj, k1 + 1);
            if (!ok) break;
            k1++;
        }
        for (int kk = k; kk <= k1; kk++)
            for (int jj = j; jj <= j1; jj++)
                memset(&g[(kk * N + jj) * N + i], 0, (size_t)(i1 - i + 1));

        const int lo[3] = { i, j, k }, hi[3] = { i1, j1, k1 };
        double centre[3], size[3];
        for (int a = 0; a < 3; a++) {
            centre[a] = x->gmin[a] + (lo[a] + hi[a] + 1) * 0.5 * x->gcell;
            size[a] = (hi[a] - lo[a] + 1) * x->gcell * MR_BOX_SHRINK;
        }
        float world[3];
        fleet_enu_to_world(centre, world);
        // ENU y is world -z and ENU z is world y, so the extents permute with
        // the centre.
        Matrix m = MatrixMultiply(
            MatrixScale((float)size[0], (float)size[2], (float)size[1]),
            MatrixTranslate(world[0], world[1], world[2]));
        bucket_push(x->chunk, B_FREE, m);
    }
}

// Is every face neighbour of this carved cell also carved?
//
// Such a cell is enclosed by its own neighbours from every direction, so it can
// never be seen: it contributes nothing but overdraw. Dropping the interior and
// keeping only the boundary -- the frontier against unknown space, and the skin
// against surfaces -- draws the same volume with far fewer cells.
static bool free_interior(const octomap_t *m, const om_leaf_t *leaf) {
    for (int axis = 0; axis < 3; axis++) {
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            double p[3] = { leaf->center[0], leaf->center[1], leaf->center[2] };
            p[axis] += sgn * leaf->size;
            if (octomap_query(m, p[0], p[1], p[2]) != OM_FREE) return false;
        }
    }
    return true;
}

static void extract_leaf(const om_leaf_t *leaf, void *user) {
    extract_ctx_t *x = (extract_ctx_t *)user;
    if (leaf->state == OM_UNKNOWN) return;

    const bool occupied = (leaf->state == OM_OCCUPIED);
    if (!occupied) {
        if (!x->r->show_free) return;
        if (!x->r->free_interior && free_interior(x->map, leaf)) return;
        if (x->grid) { mesh_mark(x, leaf); return; }
    }

    int bucket;
    switch (x->r->mode) {
        case MAP_DRAW_COVERAGE:
            bucket = occupied ? B_OCC_STRONG : B_FREE;
            break;
        case MAP_DRAW_DIVERGENCE:
            if (!occupied) return;   // divergence is about surfaces
            bucket = om_node_contested(x->map, leaf->node) ? B_CONTESTED : B_OCC_WEAK;
            break;
        case MAP_DRAW_CONTRIBUTION: {
            if (!occupied) return;
            const uint32_t obs = leaf->node->observers;
            const bool mine = (obs & x->focus_mask) != 0;
            const bool others = (obs & ~x->focus_mask) != 0;
            if (mine && others) bucket = B_SHARED;
            else if (mine)      bucket = B_FOCUS;
            else                bucket = B_OCC_WEAK;
            break;
        }
        default:
            if (!occupied) bucket = B_FREE;
            else bucket = (abs((int)leaf->node->log_odds) >= OM_LO_CLAMP / 2)
                        ? B_OCC_STRONG : B_OCC_WEAK;
            break;
    }

    float world[3];
    fleet_enu_to_world(leaf->center, world);
    const float s = (float)leaf->size * MR_CELL_SHRINK;
    Matrix m = MatrixMultiply(MatrixScale(s, s, s),
                              MatrixTranslate(world[0], world[1], world[2]));
    bucket_push(x->chunk, bucket, m);
}

static int lod_for_distance(const octomap_t *map, float distance_m) {
    // Roughly: keep cells above a couple of pixels. Each halving of resolution
    // costs one depth level, so step the cap with distance.
    int depth = map->max_depth;
    float d = distance_m;
    while (d > 40.0f && depth > map->chunk_depth) { depth--; d *= 0.5f; }
    return depth;
}

static void extract_chunk(map_render_t *r, map_session_t *ms, map_chunk_t *c, int lod) {
    for (int b = 0; b < MAP_RENDER_BUCKETS; b++) c->count[b] = 0;

    extract_ctx_t x;
    memset(&x, 0, sizeof(x));
    x.r = r;
    x.chunk = c;
    x.map = &ms->map;
    x.focus_mask = map_session_focus_mask(ms);
    if (x.focus_mask == 0) x.focus_mask = om_vehicle_bit(0);

    const bool draws_free = r->show_free
        && (r->mode == MAP_DRAW_OCCUPANCY || r->mode == MAP_DRAW_COVERAGE);
    const int levels = lod - ms->map.chunk_depth;
    if (r->free_mesh && draws_free && levels >= 0 && levels <= MR_MESH_LEVELS_MAX) {
        double centre[3], size;
        octomap_chunk_center(&ms->map, c->key, centre, &size);
        x.gdim = 1 << levels;
        x.gcell = size / (double)x.gdim;
        for (int a = 0; a < 3; a++) x.gmin[a] = centre[a] - size * 0.5;
        x.grid = r->mesh_grid;
        memset(x.grid, 0, (size_t)x.gdim * x.gdim * x.gdim);
    }

    octomap_iterate_chunk_lod(&ms->map, c->key, lod, extract_leaf, &x);
    if (x.grid) mesh_flush(&x);
    c->lod_depth = lod;
    r->stats.chunks_extracted++;
}

// ---------------------------------------------------------------- setup

int map_render_init(map_render_t *r) {
    if (!r) return -1;
    memset(r, 0, sizeof(*r));
    r->mode = MAP_DRAW_OCCUPANCY;
    r->visible = true;
    r->show_free = true;
    r->free_mesh = true;
    r->max_draw_distance_m = 400.0f;
    r->extract_budget = 24;
    // Frame 0 is never a valid "drawn this frame" stamp, because a fresh chunk
    // is zeroed and would otherwise claim to have been drawn already.
    r->frame = 1;

    if (!table_alloc(r, MR_INITIAL_CHUNKS)) return -1;

    // One grid serves every chunk, because extraction is not re-entrant.
    const size_t grid_bytes = (size_t)MR_MESH_DIM_MAX * MR_MESH_DIM_MAX * MR_MESH_DIM_MAX;
    r->mesh_grid = (uint8_t *)malloc(grid_bytes);
    if (!r->mesh_grid) r->free_mesh = false;   // a cube per cell still draws

    r->cube = GenMeshCube(1.0f, 1.0f, 1.0f);

    // Instancing needs a shader that consumes the per-instance transform. If it
    // is unavailable we fall back to immediate cubes: slower, but the map still
    // draws rather than vanishing.
    char vs[512], fs[512];
    asset_path("shaders/map_instanced.vs", vs, sizeof(vs));
    asset_path("shaders/map_instanced.fs", fs, sizeof(fs));
    r->shader = LoadShader(vs, fs);
    if (r->shader.id != 0) {
        r->shader.locs[SHADER_LOC_MATRIX_MVP] = GetShaderLocation(r->shader, "mvp");
        r->shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocationAttrib(r->shader, "instanceTransform");
        r->shader.locs[SHADER_LOC_COLOR_DIFFUSE] = GetShaderLocation(r->shader, "colDiffuse");
        r->instanced = (r->shader.locs[SHADER_LOC_MATRIX_MODEL] != -1);
    }

    for (int b = 0; b < MAP_RENDER_BUCKETS; b++) {
        r->material[b] = LoadMaterialDefault();
        if (r->instanced) r->material[b].shader = r->shader;
    }
    r->ready = true;
    return 0;
}

void map_render_free(map_render_t *r) {
    if (!r || !r->ready) return;
    for (uint32_t i = 0; i < r->chunk_cap; i++)
        if (r->chunks[i].used) chunk_release(&r->chunks[i]);
    free(r->chunks);
    r->chunks = NULL;
    free(r->mesh_grid);
    r->mesh_grid = NULL;
    UnloadMesh(r->cube);
    if (r->shader.id != 0) UnloadShader(r->shader);
    r->ready = false;
}

void map_render_invalidate(map_render_t *r) {
    if (!r || !r->chunks) return;
    for (uint32_t i = 0; i < r->chunk_cap; i++)
        if (r->chunks[i].used) r->chunks[i].lod_depth = -1;
}

void map_render_cycle_mode(map_render_t *r, int delta) {
    if (!r) return;
    int m = (int)r->mode + delta;
    while (m < 0) m += MAP_DRAW_MODE_COUNT;
    r->mode = (map_draw_mode_t)(m % MAP_DRAW_MODE_COUNT);
    map_render_invalidate(r);
}

// ---------------------------------------------------------------- draw

// Palette is fixed rather than themed: free / unknown / occupied and contested
// have to stay distinguishable whichever theme is loaded, and a theme that
// happened to reuse the contested pink would hide a real signal.
static Color bucket_colour(const map_render_t *r, int bucket, const theme_t *theme) {
    (void)r;
    (void)theme;
    switch (bucket) {
        case B_FREE:       return (Color){  46, 111, 142,  40 };
        case B_OCC_WEAK:   return (Color){ 176, 122,  58, 150 };
        case B_OCC_STRONG: return (Color){ 255, 180,  84, 235 };
        case B_CONTESTED:  return (Color){ 255,  79, 163, 245 };
        case B_FOCUS:      return (Color){  79, 195, 255, 240 };
        case B_SHARED:     return (Color){ 240, 246, 252, 220 };
        default:           return WHITE;
    }
}

// Cheap sphere-vs-frustum test using the six planes derived from the view
// projection. Chunks that fail cost nothing beyond this.
typedef struct { float p[6][4]; } frustum_t;

static void frustum_from_matrix(frustum_t *f, Matrix m) {
    const float rows[16] = {
        m.m0, m.m4, m.m8,  m.m12,
        m.m1, m.m5, m.m9,  m.m13,
        m.m2, m.m6, m.m10, m.m14,
        m.m3, m.m7, m.m11, m.m15,
    };
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 4; k++) {
            f->p[i * 2 + 0][k] = rows[12 + k] + rows[i * 4 + k];
            f->p[i * 2 + 1][k] = rows[12 + k] - rows[i * 4 + k];
        }
    }
    for (int i = 0; i < 6; i++) {
        const float n = sqrtf(f->p[i][0] * f->p[i][0] + f->p[i][1] * f->p[i][1]
                            + f->p[i][2] * f->p[i][2]);
        if (n > 1e-6f) for (int k = 0; k < 4; k++) f->p[i][k] /= n;
    }
}

static bool frustum_sphere(const frustum_t *f, Vector3 c, float radius) {
    for (int i = 0; i < 6; i++) {
        const float d = f->p[i][0] * c.x + f->p[i][1] * c.y + f->p[i][2] * c.z + f->p[i][3];
        if (d < -radius) return false;
    }
    return true;
}

static void draw_bucket(map_render_t *r, map_chunk_t *chunk, int b,
                        const theme_t *theme) {
    const Color col = bucket_colour(r, b, theme);
    r->stats.instances_drawn += (uint32_t)chunk->count[b];
    r->stats.draw_calls++;

    if (r->instanced) {
        r->material[b].maps[MATERIAL_MAP_DIFFUSE].color = col;
        DrawMeshInstanced(r->cube, r->material[b], chunk->xf[b], chunk->count[b]);
    } else {
        for (int i = 0; i < chunk->count[b]; i++) {
            const Matrix *m = &chunk->xf[b][i];
            const Vector3 p = { m->m12, m->m13, m->m14 };
            DrawCube(p, m->m0, m->m5, m->m10, col);
        }
    }
}

void map_render_draw(map_render_t *r, map_session_t *ms, Camera3D camera,
                     const theme_t *theme) {
    if (!r || !r->ready || !r->visible || !ms) return;

    memset(&r->stats, 0, sizeof(r->stats));
    const double t0 = GetTime();

    const int sw = GetScreenWidth(), sh = GetScreenHeight();
    const float aspect = (sh > 0) ? (float)sw / (float)sh : 1.0f;
    Matrix view = MatrixLookAt(camera.position, camera.target, camera.up);
    // The fullscreen ortho views hand in an orthographic camera, where raylib
    // reuses fovy as a world-space span in metres rather than an angle.
    // Feeding that span to MatrixPerspective gives tan(half-span) of a number
    // far past 90 degrees and a degenerate matrix that culls the entire map,
    // so the projection has to match what BeginMode3D will actually issue.
    const bool ortho = (camera.projection == CAMERA_ORTHOGRAPHIC);
    Matrix proj;
    if (ortho) {
        const double top = camera.fovy * 0.5;
        const double right = top * aspect;
        proj = MatrixOrtho(-right, right, -top, top,
                           RL_CULL_DISTANCE_NEAR, RL_CULL_DISTANCE_FAR);
    } else {
        proj = MatrixPerspective(camera.fovy * DEG2RAD, aspect,
                                 RL_CULL_DISTANCE_NEAR, RL_CULL_DISTANCE_FAR);
    }
    frustum_t frustum;
    frustum_from_matrix(&frustum, MatrixMultiply(view, proj));

    // Retire cached chunks the map no longer has, then walk the live set.
    uint32_t cursor = 0;
    int64_t key;
    int extracted = 0;

    bool all_latched = true;

    while (octomap_chunk_next(&ms->map, &cursor, &key)) {
        r->stats.chunks_live++;

        double centre[3], size;
        octomap_chunk_center(&ms->map, key, centre, &size);
        float world[3];
        fleet_enu_to_world(centre, world);
        const Vector3 c = { world[0], world[1], world[2] };
        const float radius = (float)size * 0.87f;   // half diagonal

        const float dx = c.x - camera.position.x;
        const float dy = c.y - camera.position.y;
        const float dz = c.z - camera.position.z;
        const float dist = sqrtf(dx * dx + dy * dy + dz * dz);

        // Find (or create) the cache entry before any culling, so a chunk that
        // is off-screen this frame still latches the map's all-dirty flag. A
        // single global flag cannot be the re-extract signal for a set the
        // renderer only partially visits: cull first and the chunks behind the
        // camera would report clean forever and keep drawing pre-prune
        // geometry.
        map_chunk_t *chunk = chunk_find(r, key, true);
        if (!chunk) { all_latched = false; continue; }
        if (octomap_chunk_is_dirty(&ms->map, key)) chunk->needs_extract = true;

        // In ortho there is no meaningful eye distance -- the camera sits an
        // arbitrary way back along the view axis and everything in the slab is
        // the same size on screen -- so only the frustum decides.
        if (!ortho && dist - radius > r->max_draw_distance_m) continue;
        if (!frustum_sphere(&frustum, c, radius)) continue;

        chunk->center = c;
        chunk->size = (float)size;

        const int lod = ortho ? lod_for_distance(&ms->map, 0.0f)
                              : lod_for_distance(&ms->map, dist);
        const bool dirty = chunk->needs_extract;
        if ((dirty || chunk->lod_depth != lod) && extracted < r->extract_budget) {
            extract_chunk(r, ms, chunk, lod);
            octomap_chunk_clear_dirty(&ms->map, key);
            chunk->needs_extract = false;
            extracted++;
        } else if (chunk->lod_depth < 0) {
            continue;   // never extracted and out of budget this frame
        }

        r->stats.chunks_drawn++;
        chunk->drawn_frame = r->frame;

        // Surfaces first, and only surfaces. Carved space is translucent and is
        // drawn in a second pass below.
        for (int b = 0; b < MAP_RENDER_BUCKETS; b++) {
            if (b == B_FREE || chunk->count[b] == 0) continue;
            draw_bucket(r, chunk, b, theme);
        }
    }

    // Carved space last. Ordering is the whole fix.
    //
    // Free cells are 16% alpha and they write depth like anything else, so a
    // pane of empty air can z-reject the surface behind it: the cell in front
    // is barely visible but it still owns the depth buffer. B_FREE is bucket 0,
    // so it was drawn *before* the surfaces in its own chunk, and chunks are
    // walked in hash order -- which parts of a surface survived was decided by
    // a hash. Measured on statue-fleet, every drawable leaf was reaching the
    // GPU (380,798 instances against 380,798 in the map) while most of the
    // statue was missing from the picture, which is what sent the first look at
    // this chasing an extraction backlog that does not exist.
    //
    // Drawing surfaces first fixes it, and the depth writes have to *stay on*
    // here. They are what limits the veil to roughly its nearest layer; turning
    // them off lets every layer of a 70 m column of carved air blend in turn
    // until the volume is opaque. Scored against a free-hidden reference frame,
    // as the share of surface pixels that still read as surface:
    //
    //     free first, depth writes on (the bug)      34.2%
    //     surfaces first, depth writes off           50.3%
    //     surfaces first, depth writes on            67.9%
    //     ... and skipping enclosed free cells       72.2%
    //
    // What the write ordering does *not* fix is which of the nearer free cells
    // blend before the nearest one wins the depth test, so the veil's exact
    // shade still depends on draw order. That is a shade, not a surface: no
    // ordering can hide geometry now that the surfaces own the depth buffer
    // first.
    //
    // Flushing matters for the non-instanced fallback, where DrawCube goes
    // through rlgl's batch: without it the veil could be rasterised alongside
    // the surfaces rather than after them, and the ordering would buy nothing.
    rlDrawRenderBatchActive();
    for (uint32_t i = 0; i < r->chunk_cap; i++) {
        map_chunk_t *chunk = &r->chunks[i];
        if (!chunk->used || chunk->drawn_frame != r->frame) continue;
        if (chunk->count[B_FREE] == 0) continue;
        draw_bucket(r, chunk, B_FREE, theme);
    }
    r->frame++;

    // The all-dirty flag is cleared once every live chunk has latched it into
    // its own needs_extract bit -- not once the visible ones have re-extracted.
    // Extraction is budgeted and culled; latching is neither, so this is the
    // only point at which the flag has genuinely been consumed by everyone.
    if (ms->map.all_dirty && all_latched) ms->map.all_dirty = false;

    r->stats.extract_ms = (float)((GetTime() - t0) * 1000.0);

}
