#ifndef TEST_ORTHO_RENDER_H
#define TEST_ORTHO_RENDER_H

// Orthographic views of a built map, rendered straight from the octree.
//
// These are the fixtures' visual evidence: free / unknown / occupied have to be
// distinguishable, contested cells have to stand out, and a focused vehicle's
// contribution has to be separable from the fleet's. Rendering from the map
// data (rather than screenshotting the viewer) keeps it reproducible in CI and
// on a machine with no GPU.

#include "canvas.h"
#include "geom.h"
#include "octomap.h"
#include "timeline.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ORTHO_TOP = 0,   // looking down: east right, north up
    ORTHO_SIDE,      // looking north: east right, up up
    ORTHO_FRONT,     // looking east: north right, up up
} ortho_view_t;

typedef enum {
    ORTHO_OCCUPANCY = 0,   // log-odds -> opacity
    ORTHO_COVERAGE,        // observed vs unobserved
    ORTHO_DIVERGENCE,      // contested cells
    ORTHO_CONTRIBUTION,    // the focused vehicle's cells
} ortho_mode_t;

typedef struct {
    double        min[3], max[3];    // world bounds, session ENU metres
    uint32_t      focus_mask;        // contribution highlight
    const geom_scene_t *truth;       // optional: true geometry, drawn as an outline
    double        truth_time_s;
    const timeline_t *tracks;        // optional: vehicle paths
    int           track_count;
    const char   *subtitle;
} ortho_opts_t;

// Fit bounds around everything the map knows about, with a margin.
void ortho_auto_bounds(const octomap_t *map, double margin_m, double min[3], double max[3]);

void ortho_draw_panel(canvas_t *c, int x0, int y0, int w, int h,
                      const octomap_t *map, ortho_view_t view, ortho_mode_t mode,
                      const ortho_opts_t *o, const char *title);

const char *ortho_mode_name(ortho_mode_t m);
const char *ortho_view_name(ortho_view_t v);

// Per-vehicle track colour, shared with the legend so the two always agree.
uint32_t ortho_vehicle_colour(int index);

#ifdef __cplusplus
}
#endif

#endif
