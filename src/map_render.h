#ifndef MAP_RENDER_H
#define MAP_RENDER_H

// Chunked renderer for the fleet map.
//
// The tree is chunked at a fixed depth (8 m by default) and each chunk keeps
// its own GPU instance buffer. Only dirty chunks are re-extracted, never the
// whole map, and chunks are frustum-culled before they cost anything. Octree
// depth doubles as level of detail: distant chunks are drawn from coarse nodes,
// which is the second reason the adaptive structure was the right call.

#include <stdbool.h>
#include <stdint.h>

#include "raylib.h"
#include "map_session.h"
#include "octomap.h"
#include "theme.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MAP_DRAW_OCCUPANCY = 0,   // log-odds -> opacity
    MAP_DRAW_COVERAGE,        // observed vs unobserved: "what did we look at"
    MAP_DRAW_DIVERGENCE,      // contested cells
    MAP_DRAW_CONTRIBUTION,    // the focused vehicle's cells
    MAP_DRAW_MODE_COUNT
} map_draw_mode_t;

const char *map_draw_mode_name(map_draw_mode_t m);

// Instances are grouped into a few colour buckets so one instanced draw call
// covers many cells; raylib's instancing has no per-instance colour.
#define MAP_RENDER_BUCKETS 6

typedef struct {
    int64_t  key;
    bool     used;
    Vector3  center;      // world (Raylib) coordinates
    float    size;
    int      lod_depth;   // depth this chunk was extracted at
    // Latched from the map's dirty bits before culling, so a chunk that is
    // off-screen when the map changes still rebuilds when it comes back.
    bool     needs_extract;
    // Frame this chunk last passed culling, so the translucent second pass can
    // revisit exactly the chunks the first pass drew without a second walk of
    // the map's chunk index.
    uint32_t drawn_frame;
    Matrix  *xf[MAP_RENDER_BUCKETS];
    int      count[MAP_RENDER_BUCKETS];
    int      cap[MAP_RENDER_BUCKETS];
} map_chunk_t;

typedef struct {
    uint32_t chunks_live;
    uint32_t chunks_drawn;
    uint32_t chunks_extracted;   // this frame
    uint32_t instances_drawn;
    uint32_t draw_calls;
    float    extract_ms;
} map_render_stats_t;

typedef struct {
    map_chunk_t *chunks;
    uint32_t     chunk_cap;      // power of two, open addressing on key
    uint32_t     chunk_count;

    Mesh      cube;
    Shader    shader;
    bool      instanced;
    Material  material[MAP_RENDER_BUCKETS];
    bool      ready;

    map_draw_mode_t mode;
    bool      visible;
    bool      show_free;
    // Draw carved cells that are enclosed by other carved cells. Off by
    // default: they are invisible by construction and cost only overdraw.
    // Turning it on with free_mesh also on is measurably worse on both counts
    // -- the interior gives the merge more to swallow, so the veil ends up
    // solider *and* larger.
    bool      free_interior;
    // Merge contiguous carved cells into boxes rather than drawing a cube per
    // cell. Off leaves the cube-per-cell path, which is also what runs when a
    // chunk is too deep to rasterise.
    bool      free_mesh;
    uint8_t  *mesh_grid;         // scratch for the merge, one chunk's worth
    float     max_draw_distance_m;
    int       extract_budget;    // chunks re-extracted per frame
    uint32_t  frame;             // draw counter, for drawn_frame above

    map_render_stats_t stats;
} map_render_t;

int  map_render_init(map_render_t *r);
void map_render_free(map_render_t *r);

// Re-extract dirty and stale-LOD chunks, then draw. Call inside BeginMode3D.
void map_render_draw(map_render_t *r, map_session_t *ms, Camera3D camera,
                     const theme_t *theme);

// Drop every cached chunk; used after a scrub rebuilds the map wholesale.
void map_render_invalidate(map_render_t *r);

void map_render_cycle_mode(map_render_t *r, int delta);

#ifdef __cplusplus
}
#endif

#endif
