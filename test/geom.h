#ifndef TEST_GEOM_H
#define TEST_GEOM_H

// Bounded plane patches: the only primitive the fixtures need, and enough to
// build a ground, a wall, a corridor and a box. Because the geometry is known,
// map error is a number rather than an impression.

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GEOM_MAX_PLANES 16

typedef struct {
    double point[3];      // patch centre, session ENU metres
    double normal[3];     // unit normal
    double u[3], v[3];    // orthonormal in-plane axes
    double half_u, half_v;
    double active_from_s; // the patch exists on [from, to); `vanishing` uses this
    double active_to_s;
    char   label[24];
} geom_plane_t;

typedef struct {
    geom_plane_t planes[GEOM_MAX_PLANES];
    int          count;
} geom_scene_t;

// Build an axis-bounded patch. `normal` need not be unit; in-plane axes are
// derived from it.
void geom_plane_make(geom_plane_t *p, const double point[3], const double normal[3],
                     double half_u, double half_v, const char *label);

void geom_plane_set_lifetime(geom_plane_t *p, double from_s, double to_s);

static inline bool geom_plane_active(const geom_plane_t *p, double t_s) {
    return t_s >= p->active_from_s && t_s < p->active_to_s;
}

// Nearest hit along the ray within [0, max_t]. Returns false when the ray
// misses every active patch.
bool geom_scene_raycast(const geom_scene_t *s, double t_s, const double origin[3],
                        const double dir[3], double max_t, double *t_hit, int *plane_idx);

// Distance from a point to the nearest active patch, clamped to the patch
// bounds. Returns a large value when nothing is active.
double geom_scene_distance(const geom_scene_t *s, double t_s, const double p[3]);

// True when the point lies on an active patch within `tol` metres.
bool geom_scene_on_surface(const geom_scene_t *s, double t_s, const double p[3], double tol);

#ifdef __cplusplus
}
#endif

#endif
