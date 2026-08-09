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
    // Which sub-case of the fixture cast this ray. `orientations` sets it to
    // the mount index, so the checker can score each of the 41 mounts on its
    // own instead of averaging one bad entry away against forty good ones.
    // 0 for fixtures with nothing to group by.
    uint8_t group;
    uint8_t pad;
} truth_ray_t;

typedef struct {
    double  surface_rms_max_m;
    double  false_occupied_max;    // fraction of occupied cells off-surface
    double  false_free_max;        // fraction of surface samples reported free
    double  coverage_min;          // fraction of swept volume that is not unknown
    int64_t occupied_cells_min;
    int64_t occupied_cells_max;    // -1 = unbounded
    double  memory_plateau_ratio;  // late/early live-node ratio ceiling, 0 = unchecked
    // Absolute bounds. A self-referential ratio cannot notice pruning being
    // switched off -- the map just grows uniformly and the ratio improves. An
    // absolute ceiling can, and a deterministic fixture can assert one.
    int64_t live_nodes_max;        // 0 = unchecked
    int64_t prune_blocks_min;      // blocks pruning must have reclaimed, 0 = unchecked
    // Ceiling on how many times the prune pass may run across the whole replay.
    //
    // A deterministic counter, which a wall-clock throughput floor is not: the
    // regression this guards against is a full-tree walk per drain, and on this
    // machine that is 13,617 passes against 132. Asserting the count rather
    // than the resulting rays/s means the check has the same teeth on a slow CI
    // runner as on a fast workstation. 0 = unchecked.
    int64_t prune_passes_max;
    double  min_rays_per_s;        // 0 = unchecked
    int     require_drops;         // 1 = drops must be non-zero and reported
    double  contested_max_dist_m;  // contested cells must sit within this of the offset band
    int     require_contested;     // 1 = contested cells must exist
    // Divergence flagged *everywhere* is as useless as divergence flagged
    // nowhere, and a floor alone cannot tell the two apart. Expressed as a
    // share of occupied cells so it survives a change of fixture scale.
    double  contested_share_max;   // 0 = unchecked
    double  cone_ratio_min;        // mean occupied cell size, wide FOV / narrow FOV
    double  weak_ratio_min;        // mean |log-odds|, clean / weak
    // Per-vehicle surface RMS bounds. The clock fixture uses both: vehicle 0
    // flies with its ranging and pose streams on one clock and must come out
    // clean, vehicle 1 flies them skewed and must come out visibly worse. If
    // the viewer paired every range with the latest attitude instead of the one
    // at the range's own timestamp, both would smear and the pair would fail.
    double  vehicle0_rms_max;
    double  vehicle1_rms_min;
    // Worst single group's false-free rate. A pooled average cannot see one
    // wrong orientation-table entry among forty right ones; this can.
    double  group_false_free_max;   // 0 = unchecked
    int     group_min_rays;         // groups thinner than this are not scored
    // The cooperative claim, as two numbers. `merged` is the share of all
    // observed surface that the fleet's map holds; `solo` is the share the
    // best-performing single drone contributed. A floor on the first without a
    // ceiling on the second proves only that somebody mapped it.
    double  merged_surface_min;     // 0 = unchecked
    double  solo_surface_max;       // 0 = unchecked
    // Does the map look like the thing it mapped? Intersection over union
    // between the map's occupied cells and the world voxelised at the same
    // resolution. A silhouette that is merely *near* the object scores well on
    // surface RMS and badly here, which is the point.
    double  shape_iou_min;          // 0 = unchecked
    double  shape_recall_min;       // share of the observed envelope the map keeps
    double  shape_precision_min;    // share of the map's voxels that are on the object
    // Side of the comparison lattice. Both the map and the world are voxelised
    // on it, so the score is between two sets of the same cells. It is coarser
    // than the map's leaf on purpose: below about 0.7 m the reference cloud's
    // own splat spacing and the sensor's cone footprint dominate, and a finer
    // lattice would measure those rather than the map's fidelity. 0 = leaf.
    double  shape_voxel_m;
    // Physical tolerance for "is this surface point represented". Zero keeps
    // the historical behaviour of one map leaf, which is resolution-relative
    // and so asks a 7.8 mm map a hundred times harder a question than a 0.25 m
    // one. A fixture chasing centimetres states the tolerance in metres.
    double  surface_tol_m;
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

    // A splat world, when the fixture ranged against one instead of planes.
    // The checker loads the identical file rather than being told the answer,
    // so the world the map is scored against is the world it was built from.
    char     splat_path[384];
    double   splat_origin_enu[3];
    // The exact triangles the splats were sampled from. A splat cloud cannot
    // verify a map to a finer tolerance than its own splat spacing, so
    // centimetre fixtures score against these instead.
    char     mesh_path[384];

    // Map configuration the checker must reproduce for the numbers to mean
    // anything.
    double   root_size_m;
    int      max_depth;
    int      coarse_depth;
    double   skip_near_m;
    double   refine_dist_m;   // 0 -> the octomap default
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
