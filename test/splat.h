#ifndef TEST_SPLAT_H
#define TEST_SPLAT_H

// A Gaussian splat cloud, used as a *world* for the fixtures to range against.
//
// Every fixture until now ranged against analytic planes. Planes are the right
// model for walls and floors and they make the error analysis exact, but they
// cannot answer the question "does this map look like the thing it mapped" --
// every plane looks like every other plane. An object can, and a splat is the
// form a real capture of an object arrives in.
//
// Nothing here is linked into the viewer. Hawkeye never learns what a splat is:
// the fixture ray-casts against the Gaussians and emits DISTANCE_SENSOR and
// OBSTACLE_DISTANCE over the wire exactly as a vehicle does, so the ingest path
// under test is still the real one.
//
// Depth is the 3DGS formulation with a sensor's threshold rather than a
// renderer's: composite the Gaussians front to back along the ray and take the
// depth at which accumulated opacity first passes SPLAT_SURFACE_ALPHA. Unlike
// "nearest centre" this respects the fact that a Gaussian is a soft blob, and
// unlike the rendering convention of one half it does not turn every grazing
// ray into a no-return -- which would have the fixture carving free space
// through the very object it is ranging.

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pos[3];
    float alpha;
    // Inverse covariance, packed symmetric: xx, xy, xz, yy, yz, zz.
    float inv_cov[6];
    float radius;      // 3-sigma extent, for the grid
    uint8_t col[4];
} splat_t;

typedef struct {
    splat_t *s;
    uint32_t count;

    double lo[3], hi[3];

    // Uniform grid over the bounding box.
    double   cell;
    int      dim[3];
    uint32_t *bucket_start;   // dim product + 1
    uint32_t *bucket_item;    // count entries
} splat_cloud_t;

int  splat_load(splat_cloud_t *c, const char *path, char *err, size_t err_len);
void splat_free(splat_cloud_t *c);

// Front-to-back composite along the ray. Returns true when accumulated opacity
// reaches `SPLAT_SURFACE_ALPHA` within max_dist, writing the depth there.
bool splat_raycast(const splat_cloud_t *c, const double o[3], const double d[3],
                   double max_dist, double *out_t);

// Distance from a point to the nearest splat centre. Used to score occupied
// cells against the world, the way geom_scene_distance does for planes.
double splat_distance(const splat_cloud_t *c, const double p[3]);

// Does any splat centre fall inside the axis-aligned cell of side `size`
// centred on `p`? This is the reference occupancy the map is compared against.
bool splat_cell_occupied(const splat_cloud_t *c, const double p[3], double size);

// Detection threshold, not the median depth. Median depth (0.5) is the right
// definition for *rendering* a splat, but it is wrong for a range sensor: a ray
// grazing the silhouette never accumulates half its opacity, so it would be
// reported as a no-return and the fixture would then carve free space straight
// through solid statue. A real lidar returns off a grazing surface. A quarter
// of accumulated opacity is enough to say something is there.
#define SPLAT_SURFACE_ALPHA 0.25

#ifdef __cplusplus
}
#endif

#endif
