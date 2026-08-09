#ifndef TEST_TRIMESH_H
#define TEST_TRIMESH_H

// The exact surface behind a splat world.
//
// A Gaussian splat cloud is what the *sensor* sees, and it is the right world
// model for that: soft, occluding, with a depth that depends on incidence. It
// is the wrong thing to *score* against once you care about centimetres,
// because its own resolution is the splat spacing -- 34 cm for the statue
// cloud. You cannot check a map to a tolerance finer than the reference you
// check it against.
//
// So the same bake writes the placed triangles alongside the splats, and this
// answers the one question the scorer actually needs: how far is this point
// from the true surface. Exact point-triangle distance, no sampling, no
// discretisation.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float (*tri)[9];       // count triangles, 3 vertices each
    uint32_t count;

    double lo[3], hi[3];

    // Uniform grid of triangle references, so a lookup touches a handful.
    double    cell;
    int       dim[3];
    uint32_t *bucket_start;
    uint32_t *bucket_item;
} trimesh_t;

int  trimesh_load(trimesh_t *m, const char *path, char *err, size_t err_len);
void trimesh_free(trimesh_t *m);

// Exact distance from p to the nearest point on the nearest triangle.
double trimesh_distance(const trimesh_t *m, const double p[3]);

// Nearest ray-triangle intersection, Moller-Trumbore. The precision fixture
// ranges against this rather than against the splats: a Gaussian cloud is a
// soft surface whose apparent depth shifts with incidence by roughly its splat
// spacing, so ranging against one puts a ~10 cm floor under the measurable
// error however narrow the beam. A real lidar looking at a real statue sees a
// hard surface; the softness is an artifact of the representation, not of the
// world.
bool trimesh_raycast(const trimesh_t *m, const double o[3], const double d[3],
                     double max_dist, double *out_t);

#ifdef __cplusplus
}
#endif

#endif
