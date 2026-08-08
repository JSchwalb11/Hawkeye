#ifndef TEST_TRUTH_H
#define TEST_TRUTH_H

// Ground truth published beside the tlog.
//
// Two files share a prefix:
//   <prefix>.json  geometry, poses summary, map configuration and thresholds
//   <prefix>.rays  every ray the injector actually cast, as it truly was
//
// The point of publishing the rays is that the checker never has to re-derive
// what the fixture did. It scores the map against what was genuinely swept,
// which is the difference between "the picture looks right" and a number.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "geom.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRUTH_MAX_VEHICLES 32
#define TRUTH_RAYS_MAGIC 0x54525931u   /* "TRY1" */

typedef struct {
    double  t_s;
    float   origin[3];     // true sensor origin, session ENU
    float   endpoint[3];   // true endpoint: the surface hit, or the max-range point
    uint8_t vehicle;
    uint8_t hit;           // 0 == no return
    uint8_t pad[2];
} truth_ray_t;

typedef struct {
    double  surface_rms_max_m;
    double  false_occupied_max;    // fraction of occupied cells off-surface
    double  false_free_max;        // fraction of surface samples reported free
    double  coverage_min;          // fraction of swept volume that is not unknown
    int64_t occupied_cells_min;
    int64_t occupied_cells_max;    // -1 = unbounded
    double  memory_plateau_ratio;  // late/early byte ratio ceiling, 0 = unchecked
    double  min_rays_per_s;        // 0 = unchecked
    int     require_drops;         // 1 = drops must be non-zero and reported
    double  contested_max_dist_m;  // contested cells must sit within this of the offset band
    int     require_contested;     // 1 = contested cells must exist
    double  cone_ratio_min;        // mean occupied cell size, wide FOV / narrow FOV
    double  weak_ratio_min;        // mean |log-odds|, clean / weak
    // Per-vehicle surface RMS bounds. The clock fixture uses both: vehicle 0
    // flies with its ranging and pose streams on one clock and must come out
    // clean, vehicle 1 flies them skewed and must come out visibly worse. If
    // the viewer paired every range with the latest attitude instead of the one
    // at the range's own timestamp, both would smear and the pair would fail.
    double  vehicle0_rms_max;
    double  vehicle1_rms_min;
} truth_thresholds_t;

typedef struct {
    uint8_t sysid;
    double  origin_lat, origin_lon, origin_alt;   // this vehicle's GPS_GLOBAL_ORIGIN
    double  pos_offset_enu[3];                    // deliberate localisation error, if any
    char    note[48];
} truth_vehicle_t;

typedef struct {
    char    fixture[32];
    uint32_t seed;
    double  duration_s;
    double  scale;

    double  session_lat, session_lon, session_alt;

    truth_vehicle_t vehicles[TRUTH_MAX_VEHICLES];
    int     vehicle_count;

    geom_scene_t scene;

    // Map configuration the checker must reproduce for the numbers to mean
    // anything.
    double   root_size_m;
    int      max_depth;
    int      coarse_depth;
    double   skip_near_m;
    uint32_t queue_capacity;
    uint32_t budget_per_drain;
    size_t   map_byte_cap;

    truth_thresholds_t thresholds;
} truth_header_t;

// --- Writing -----------------------------------------------------------

typedef struct {
    FILE    *rays;
    uint32_t ray_count;
    char     prefix[512];
} truth_writer_t;

int  truth_writer_open(truth_writer_t *w, const char *prefix);
int  truth_writer_ray(truth_writer_t *w, const truth_ray_t *r);
int  truth_writer_finish(truth_writer_t *w, const truth_header_t *h);

// --- Reading -----------------------------------------------------------

typedef struct {
    truth_header_t header;
    truth_ray_t   *rays;
    uint32_t       ray_count;
} truth_t;

int  truth_load(truth_t *t, const char *prefix, char *err, size_t err_len);
void truth_free(truth_t *t);

#ifdef __cplusplus
}
#endif

#endif
