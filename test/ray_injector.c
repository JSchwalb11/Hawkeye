// Synthetic ray injector.
//
// Ray-casts against known geometry and emits genuine DISTANCE_SENSOR /
// OBSTACLE_DISTANCE MAVLink messages, over the wire and/or into a tlog. Because
// the geometry is known, map error becomes a number rather than an impression.
//
// This program is a test fixture and is never linked into the viewer. It talks
// to the viewer exactly the way a vehicle does, which is the whole point: the
// ingest path under test is the real one, not a back door into the map API.
//
// Determinism: fixed timestep, seeded RNG, fixed message ordering, timestamps
// derived from the simulated clock alone. A run is byte-reproducible, so CI can
// assert exact figures.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#else
#include <ws2tcpip.h>
#endif

#include <mavlink.h>

#include "geo.h"
#include "geom.h"
#include "orientation_basis.h"
#include "tlog.h"
#include "truth.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG (M_PI / 180.0)

// Session origin: the PX4 SIH default spawn, so fixtures land somewhere real.
#define SESSION_LAT 47.397742
#define SESSION_LON 8.545594
#define SESSION_ALT 489.4

// A fixed, arbitrary wall-clock epoch. Fixed so that runs are reproducible.
#define BASE_UNIX_NS 1735689600000000000LL   /* 2025-01-01T00:00:00Z */

#define MAX_SIM_VEHICLES 16
#define OBSTACLE_DISTANCE_SECTORS 72

typedef enum {
    FX_EMPTY = 0, FX_GROUND, FX_WALL, FX_CORRIDOR, FX_ORIENTATIONS, FX_MOVING,
    FX_TWO_ORIGINS, FX_DISAGREEMENT, FX_VANISHING, FX_CONE, FX_WEAK,
    FX_CLOCKS, FX_FIREHOSE, FX_ENDURANCE, FX_COOPERATIVE, FX_COUNT
} fixture_id_t;

typedef enum { SENSOR_DISTANCE, SENSOR_OBSTACLE } sensor_kind_t;

typedef struct {
    const char   *name;
    double        duration_s;
    int           vehicles;
    double        rate_hz;
    sensor_kind_t sensor;
    double        pose_rate_divisor;   // publish pose every Nth sensor tick
    truth_thresholds_t th;
    uint32_t      queue_capacity;
    uint32_t      budget_per_drain;
    // Publish every Nth ray. The long fixtures cast millions; scoring them is
    // statistical, and a 44 MB sidecar in CI helps nobody.
    uint32_t      truth_ray_stride;
} fixture_def_t;

static const fixture_def_t k_fixtures[FX_COUNT] = {
    [FX_EMPTY] = { "empty", 12.0, 1, 20.0, SENSOR_DISTANCE, 1, {
        .surface_rms_max_m = 0, .false_occupied_max = 0, .false_free_max = 1.0,
        .coverage_min = 0.95, .occupied_cells_min = 0, .occupied_cells_max = 0,
    }, 0, 0, 0 },

    [FX_GROUND] = { "ground", 20.0, 1, 20.0, SENSOR_DISTANCE, 1, {
        .surface_rms_max_m = 0.15, .false_occupied_max = 0.02, .false_free_max = 0.02,
        .coverage_min = 0.95, .occupied_cells_min = 80, .occupied_cells_max = -1,
    }, 0, 0, 0 },

    [FX_WALL] = { "wall", 20.0, 1, 20.0, SENSOR_DISTANCE, 1, {
        .surface_rms_max_m = 0.15, .false_occupied_max = 0.02, .false_free_max = 0.02,
        .coverage_min = 0.95, .occupied_cells_min = 30, .occupied_cells_max = -1,
    }, 0, 0, 0 },

    // A 5-degree sector at 30 m genuinely cannot place a surface better than
    // about +/-1.3 m, so the thresholds allow honest cone widening. What they do
    // not allow is a rotation: an off-by-one in the sector mapping puts cells
    // many metres out and takes surface RMS with it.
    [FX_CORRIDOR] = { "corridor", 25.0, 1, 10.0, SENSOR_OBSTACLE, 1, {
        .surface_rms_max_m = 0.80, .false_occupied_max = 0.30, .false_free_max = 0.05,
        .coverage_min = 0.95, .occupied_cells_min = 400, .occupied_cells_max = -1,
    }, 0, 0, 0 },

    // Every mount is scored on its own. A pooled RMS cannot see one wrong
    // table entry among forty right ones -- a 10-degree slip in entry 38, the
    // one entry whose angles are not multiples of 45 and so the likeliest
    // transcription error, moves the pooled figure by nothing at all.
    [FX_ORIENTATIONS] = { "orientations", 42.0, 1, 20.0, SENSOR_DISTANCE, 1, {
        .surface_rms_max_m = 0.15, .false_occupied_max = 0.02, .false_free_max = 0.02,
        .coverage_min = 0.95, .occupied_cells_min = 50, .occupied_cells_max = -1,
        .group_false_free_max = 0.10, .group_min_rays = 8,
    }, 0, 0, 0 },

    [FX_MOVING] = { "moving", 25.0, 1, 20.0, SENSOR_DISTANCE, 2, {
        .surface_rms_max_m = 0.20, .false_occupied_max = 0.03, .false_free_max = 0.03,
        .coverage_min = 0.95, .occupied_cells_min = 90, .occupied_cells_max = -1,
    }, 0, 0, 0 },

    [FX_TWO_ORIGINS] = { "two-origins", 20.0, 2, 20.0, SENSOR_DISTANCE, 1, {
        .surface_rms_max_m = 0.15, .false_occupied_max = 0.02, .false_free_max = 0.02,
        .coverage_min = 0.95, .occupied_cells_min = 60, .occupied_cells_max = -1,
    }, 0, 0, 0 },

    // Half the occupancy here is deliberately wrong -- that is the fixture. The
    // assertion that matters is that the contradiction is *flagged*, and that
    // the contested cells sit where the offset puts them and nowhere else.
    [FX_DISAGREEMENT] = { "disagreement", 20.0, 2, 20.0, SENSOR_DISTANCE, 1, {
        .surface_rms_max_m = 2.00, .false_occupied_max = 0.85, .false_free_max = 0.75,
        .coverage_min = 0.95, .occupied_cells_min = 50, .occupied_cells_max = -1,
        .require_contested = 1, .contested_max_dist_m = 4.0,
        .contested_share_max = 0.60,
    }, 0, 0, 0 },

    [FX_VANISHING] = { "vanishing", 75.0, 1, 10.0, SENSOR_DISTANCE, 1, {
        .surface_rms_max_m = 0.20, .false_occupied_max = 0.03, .false_free_max = 0.03,
        .coverage_min = 0.95, .occupied_cells_min = 30, .occupied_cells_max = -1,
    }, 0, 0, 0 },

    [FX_CONE] = { "cone", 20.0, 2, 20.0, SENSOR_DISTANCE, 1, {
        .surface_rms_max_m = 0.30, .false_occupied_max = 0.03, .false_free_max = 0.03,
        .coverage_min = 0.95, .occupied_cells_min = 30, .occupied_cells_max = -1,
        .cone_ratio_min = 1.5,
    }, 0, 0, 0 },

    [FX_WEAK] = { "weak", 20.0, 2, 20.0, SENSOR_DISTANCE, 1, {
        .surface_rms_max_m = 0.20, .false_occupied_max = 0.03, .false_free_max = 0.03,
        .coverage_min = 0.95, .occupied_cells_min = 40, .occupied_cells_max = -1,
        .weak_ratio_min = 1.5,
    }, 0, 0, 0 },

    // Two vehicles flying the same profile past their own wall. Vehicle 0 has
    // its ranging and pose streams on one clock; vehicle 1's ranging stamps run
    // 0.8 s ahead of its pose stamps, as a vehicle with a mixed time base does.
    // Vehicle 0 must come out flat and vehicle 1 must come out visibly smeared:
    // if the viewer paired each range with the *latest* attitude instead of the
    // one at the range's own timestamp, both would smear and this fails.
    [FX_CLOCKS] = { "clocks", 25.0, 2, 20.0, SENSOR_DISTANCE, 1, {
        .surface_rms_max_m = 0, .false_occupied_max = 1.0, .false_free_max = 1.0,
        .coverage_min = 0.90, .occupied_cells_min = 80, .occupied_cells_max = -1,
        .vehicle0_rms_max = 0.25, .vehicle1_rms_min = 0.50,
    }, 0, 0, 0 },

    [FX_FIREHOSE] = { "firehose", 12.0, 8, 20.0, SENSOR_OBSTACLE, 1, {
        .surface_rms_max_m = 0.80, .false_occupied_max = 0.30, .false_free_max = 0.10,
        .coverage_min = 0.90, .occupied_cells_min = 500, .occupied_cells_max = -1,
        .min_rays_per_s = 10000.0, .require_drops = 1,
    }, 4096, 150, 8 },

    // The cooperative claim, made falsifiable. Four bays separated by a solid
    // cross wall, one vehicle confined to each. Occlusion is real -- a vehicle
    // in the north-east bay firing south hits the cross wall, not the far
    // outer wall -- so no drone can see more than its own quarter no matter how
    // long it flies. The merged map must hold the whole structure, and no
    // single drone's contribution may. Both halves are asserted: a floor on the
    // union and a ceiling on the best individual. Together they say the fleet
    // built something none of its members could.
    [FX_COOPERATIVE] = { "cooperative", 45.0, 4, 10.0, SENSOR_OBSTACLE, 1, {
        .surface_rms_max_m = 0.80, .false_occupied_max = 0.30, .false_free_max = 0.10,
        .coverage_min = 0.85, .occupied_cells_min = 800, .occupied_cells_max = -1,
        .merged_surface_min = 0.90, .solo_surface_max = 0.45,
    }, 0, 0, 0 },

    [FX_ENDURANCE] = { "endurance", 1920.0, 2, 4.0, SENSOR_OBSTACLE, 1, {
        .surface_rms_max_m = 0.80, .false_occupied_max = 0.20, .false_free_max = 0.10,
        .coverage_min = 0.90, .occupied_cells_min = 1000, .occupied_cells_max = -1,
        .memory_plateau_ratio = 1.25,
        // The ratio alone is not enough: with pruning disabled the map grows
        // uniformly and the ratio actually improves. These two are what notice.
        .live_nodes_max = 60000, .prune_blocks_min = 100000,
    }, 0, 0, 16 },
};

// ------------------------------------------------------------ RNG

static uint32_t g_rng;
static void rng_seed(uint32_t s) { g_rng = s ? s : 0x9E3779B9u; }
static uint32_t rng_u32(void) {
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
    return g_rng;
}
// Deterministic uniform in [-1, 1].
static double rng_sym(void) { return ((double)(rng_u32() >> 8) / 8388608.0) - 1.0; }

// ------------------------------------------------------------ scene

static void add_plane(geom_scene_t *s, double px, double py, double pz,
                      double nx, double ny, double nz,
                      double hu, double hv, const char *label) {
    if (s->count >= GEOM_MAX_PLANES) return;
    const double p[3] = { px, py, pz };
    const double n[3] = { nx, ny, nz };
    geom_plane_make(&s->planes[s->count++], p, n, hu, hv, label);
}

// Which mount the `orientations` fixture is exercising right now: the 41 enum
// values plus one custom quaternion, a second each. The dwell is generous
// because the checker scores every mount separately -- ten rays is enough to
// convict one, but twenty leaves no argument about sampling.
#define ORIENT_STEP_S 1.0
static int orientation_step(double t) { return (int)(t / ORIENT_STEP_S); }

// The mount grid: one plot of ground per mount, spaced far enough apart that a
// misaimed mount cannot land on a neighbour's plot and be mistaken for it.
#define ORIENT_GRID_COLS 7
#define ORIENT_PLOT_M    8.0

static void build_scene(fixture_id_t fx, geom_scene_t *s) {
    memset(s, 0, sizeof(*s));
    switch (fx) {
        case FX_EMPTY:
            break;   // nothing in range; every ray must return max_distance

        case FX_GROUND:
        case FX_ORIENTATIONS:
            // Wide enough to hold the whole grid of per-mount plots.
            add_plane(s, 0, 0, 0, 0, 0, 1,
                      (fx == FX_GROUND) ? 40 : ORIENT_GRID_COLS * ORIENT_PLOT_M,
                      (fx == FX_GROUND) ? 40 : 6 * ORIENT_PLOT_M, "ground");
            break;

        case FX_WALL: {
            // Perpendicular to a 30-degree bearing, 15 m out. Getting the
            // body->NED rotation wrong puts the surface somewhere else entirely.
            const double bearing = 30.0 * DEG;
            const double e = sin(bearing) * 15.0, n = cos(bearing) * 15.0;
            add_plane(s, e, n, 5, -sin(bearing), -cos(bearing), 0, 12, 8, "wall");
            break;
        }

        case FX_CORRIDOR:
            add_plane(s, -4, 0, 5, 1, 0, 0, 40, 10, "west");
            add_plane(s,  4, 0, 5, -1, 0, 0, 40, 10, "east");
            break;

        case FX_MOVING:
            add_plane(s, 0, 18, 5, 0, -1, 0, 25, 8, "wall");
            break;

        case FX_TWO_ORIGINS:
        case FX_DISAGREEMENT:
            add_plane(s, 0, 20, 5, 0, -1, 0, 40, 10, "wall");
            break;

        case FX_VANISHING:
            add_plane(s, 0, 12, 5, 0, -1, 0, 6, 4, "obstacle");
            geom_plane_set_lifetime(&s->planes[0], 0.0, 30.0);
            add_plane(s, 0, 25, 5, 0, -1, 0, 30, 10, "backstop");
            break;

        case FX_CONE:
        case FX_WEAK:
            add_plane(s, -20, 12, 5, 0, -1, 0, 6, 6, "target-a");
            add_plane(s,  20, 12, 5, 0, -1, 0, 6, 6, "target-b");
            break;

        case FX_CLOCKS:
            // Oblique on purpose. Against a wall square to the sensor a pose
            // error slides the endpoint *along* the surface and hides itself;
            // at 45 degrees the same error walks straight off it, which is what
            // makes a stale-attitude pairing measurable rather than merely
            // describable.
            add_plane(s, -20, 18, 5, 0.70710678, -0.70710678, 0, 30, 10, "aligned");
            add_plane(s,  20, 18, 5, 0.70710678, -0.70710678, 0, 30, 10, "skewed");
            break;

        case FX_FIREHOSE:
        case FX_COOPERATIVE:
            // A 48 x 48 m room divided into four bays by a solid cross wall.
            // The cross is what makes the fixture mean anything: without it any
            // one vehicle could eventually see every wall and the union would
            // prove nothing.
            add_plane(s, -24, 0, 5, 1, 0, 0, 24, 5, "west");
            add_plane(s,  24, 0, 5, -1, 0, 0, 24, 5, "east");
            add_plane(s, 0, -24, 5, 0, 1, 0, 24, 5, "south");
            add_plane(s, 0,  24, 5, 0, -1, 0, 24, 5, "north");
            add_plane(s, 0, 0, 5, 1, 0, 0, 24, 5, "divider-ns");
            add_plane(s, 0, 0, 5, 0, 1, 0, 24, 5, "divider-ew");
            break;

        case FX_ENDURANCE:
            // A bounded box: the endurance fixture needs the volume closed so
            // memory has something to plateau at.
            add_plane(s, -20, 0, 8, 1, 0, 0, 20, 8, "west");
            add_plane(s,  20, 0, 8, -1, 0, 0, 20, 8, "east");
            add_plane(s, 0, -20, 8, 0, 1, 0, 20, 8, "south");
            add_plane(s, 0,  20, 8, 0, -1, 0, 20, 8, "north");
            add_plane(s, 0, 0, 0, 0, 0, 1, 20, 20, "floor");
            break;

        default: break;
    }
}

// ------------------------------------------------------------ poses

typedef struct {
    double enu[3];               // true position
    double roll, pitch, yaw;     // true attitude, radians
    double report_offset[3];     // deliberate localisation error added on the wire
} sim_pose_t;

// Nothing sits exactly on an axis. A ray running precisely along a cell
// boundary is a knife edge that no voxel map handles meaningfully, and no real
// flight produces one; putting fixtures on the grid would be testing an
// artefact rather than the map.
#define OFF_E 0.37
#define OFF_N 0.19

static void vehicle_pose(fixture_id_t fx, int veh, double t, sim_pose_t *p) {
    memset(p, 0, sizeof(*p));
    p->enu[0] = OFF_E;
    p->enu[1] = OFF_N;

    switch (fx) {
        case FX_EMPTY:
            // Drifting, so the no-return rays sweep a volume rather than a line.
            p->enu[0] += 3.0 * sin(t * 0.40);
            p->enu[1] += 3.0 * cos(t * 0.33);
            p->enu[2] = 30.1;
            break;

        case FX_GROUND:
            // Drift a little so the ground gets covered rather than sampled once.
            p->enu[0] += 6.0 * sin(t * 0.3);
            p->enu[1] += 6.0 * cos(t * 0.21);
            p->enu[2] = 12.0;
            break;

        case FX_WALL:
            p->enu[2] = 5.1;
            // Sweep across the wall so it is painted, not poked.
            p->yaw = 30.0 * DEG + 20.0 * DEG * sin(t * 0.5);
            break;

        case FX_CORRIDOR:
            p->enu[1] = -20.0 + 1.6 * t;
            p->enu[2] = 5.1;
            p->yaw = 0.4 * sin(t * 0.25);   // gentle yaw: sector mapping must follow
            break;

        case FX_ORIENTATIONS: {
            // Each mount gets its own plot of ground, far enough from its
            // neighbours that no other mount's rays can cover for it. Drifting
            // over one shared patch was the flaw in the earlier version: forty
            // correct mounts painted the same cells the wrong one was supposed
            // to, so its absence was invisible.
            const int step = orientation_step(t);
            const int col = step % ORIENT_GRID_COLS;
            const int row = step / ORIENT_GRID_COLS;
            p->enu[0] += ((double)col - (ORIENT_GRID_COLS - 1) * 0.5) * ORIENT_PLOT_M;
            p->enu[1] += ((double)row - 2.5) * ORIENT_PLOT_M;
            // A little movement inside the plot so the footprint is a few cells
            // wide and the score is not one cell's worth of quantisation.
            p->enu[0] += 0.4 * sin(t * 3.1);
            p->enu[1] += 0.4 * cos(t * 2.7);
            p->enu[2] = 10.1;
            break;   // attitude is derived per-observation from the mount under test
        }

        case FX_MOVING:
            p->enu[0] += 8.0 * sin(t * 0.25);
            p->enu[2] = 5.1;
            p->yaw = 25.0 * DEG * sin(t * 0.31);
            break;

        case FX_TWO_ORIGINS:
            p->enu[0] += (veh == 0) ? -3.0 : 3.0;
            p->enu[2] = 5.1;
            p->yaw = 20.0 * DEG * sin(t * 0.45 + veh);
            break;

        case FX_DISAGREEMENT:
            p->enu[0] += (veh == 0) ? -0.6 : 0.6;
            p->enu[2] = 5.1;
            p->yaw = 20.0 * DEG * sin(t * 0.45);
            // Vehicle 1 believes it is 1.5 m further north than it is. Its rays
            // therefore carve straight through the wall vehicle 0 sees and stop
            // 1.5 m behind it: free evidence against confident occupancy, which
            // is exactly what the map must flag as contested.
            if (veh == 1) p->report_offset[1] = 1.5;
            break;

        case FX_VANISHING:
            p->enu[2] = 5.1;
            p->yaw = 12.0 * DEG * sin(t * 0.5);
            break;

        case FX_CONE:
        case FX_WEAK:
            p->enu[0] += (veh == 0) ? -20.0 : 20.0;
            p->enu[2] = 5.1;
            p->yaw = 22.0 * DEG * sin(t * 0.45);
            break;

        case FX_CLOCKS:
            // Translating and yawing hard enough that a stale pose is obvious.
            p->enu[0] += ((veh == 0) ? -20.0 : 20.0) + 6.0 * sin(t * 0.55);
            p->enu[2] = 5.1;
            p->yaw = 22.0 * DEG * sin(t * 0.55);
            break;

        case FX_FIREHOSE:
        case FX_COOPERATIVE: {
            // One vehicle per bay, orbiting its own quarter. Nothing here ever
            // crosses into a neighbour's bay, so the partition is a property of
            // the flight as well as of the geometry.
            const double cx = (veh & 1) ? 12.0 : -12.0;
            const double cy = (veh & 2) ? 12.0 : -12.0;
            const double phase = t * 0.35 + (double)veh * 0.7;
            p->enu[0] += cx + 7.0 * cos(phase);
            p->enu[1] += cy + 7.0 * sin(phase);
            p->enu[2] = 5.0 + 0.4 * sin(t * 0.5);
            p->yaw = phase;
            break;
        }

        case FX_ENDURANCE: {
            const double phase = t * 0.05 + (double)veh * (2.0 * M_PI / 8.0);
            p->enu[0] = 10.0 * cos(phase);
            p->enu[1] = 10.0 * sin(phase);
            p->enu[2] = 6.0 + 2.0 * sin(t * 0.02 + veh);
            p->yaw = phase;
            break;
        }

        default: break;
    }
}

// ------------------------------------------------------------ sensors

typedef struct {
    uint8_t orientation;
    bool    use_quaternion;
    float   quaternion[4];      // R_body<-sensor when use_quaternion
    float   h_fov, v_fov;       // radians
    uint16_t min_cm, max_cm;
    uint8_t covariance;
    uint8_t signal_quality;
} sensor_cfg_t;


static void sensor_config(fixture_id_t fx, int veh, double t, sensor_cfg_t *c) {
    memset(c, 0, sizeof(*c));
    c->orientation = 0;
    c->min_cm = 20;
    c->max_cm = 4000;
    c->covariance = 1;
    c->signal_quality = 100;

    switch (fx) {
        case FX_EMPTY:
            c->orientation = 25;    // PITCH_270, straight down
            c->max_cm = 2000;       // the ground is 30 m away: always a no-return
            break;

        case FX_GROUND:
            c->orientation = 25;
            break;

        case FX_ORIENTATIONS: {
            const int step = orientation_step(t);
            if (step < OB_ORIENTATION_COUNT) {
                c->orientation = (uint8_t)step;
            } else {
                // MAV_SENSOR_ROTATION_CUSTOM with a populated quaternion: the
                // quaternion must win over whatever `orientation` claims.
                c->orientation = 100;
                c->use_quaternion = true;
                double q[4];
                ob_quat_from_euler(17.0 * DEG, -41.0 * DEG, 63.0 * DEG, q);
                for (int i = 0; i < 4; i++) c->quaternion[i] = (float)q[i];
            }
            break;
        }

        case FX_CONE:
            if (veh == 0) { c->h_fov = 25.0f * (float)DEG; c->v_fov = 25.0f * (float)DEG; }
            else          { c->h_fov = 1.0f * (float)DEG;  c->v_fov = 1.0f * (float)DEG; }
            break;

        case FX_WEAK:
            if (veh == 1) {
                // Degrading quality and rising variance: this return must move
                // the map less than the clean one.
                const double u = t / 20.0;
                double sq = 60.0 - 50.0 * u;
                if (sq < 10.0) sq = 10.0;
                double cov = 9.0 + 27.0 * u;
                if (cov > 36.0) cov = 36.0;
                c->signal_quality = (uint8_t)sq;
                c->covariance = (uint8_t)cov;
            }
            break;

        default: break;
    }
}

// ------------------------------------------------------------ emission

typedef struct {
    tlog_writer_t tlog;
    bool          have_tlog;
    int           sock;
    struct sockaddr_in dest;
    bool          have_udp;
    bool          realtime;
    uint64_t      messages;
} emitter_t;

static void emit(emitter_t *e, double t_s, mavlink_message_t *msg) {
    const int64_t arrival = BASE_UNIX_NS + (int64_t)(t_s * 1e9);
    if (e->have_tlog) tlog_writer_write_msg(&e->tlog, arrival, (struct __mavlink_message *)msg);
    if (e->have_udp) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
#ifndef _WIN32
        (void)sendto(e->sock, buf, len, 0, (struct sockaddr *)&e->dest, sizeof(e->dest));
#else
        (void)sendto(e->sock, (const char *)buf, len, 0,
                     (struct sockaddr *)&e->dest, sizeof(e->dest));
#endif
    }
    e->messages++;
}

// ------------------------------------------------------------ simulation

typedef struct {
    fixture_id_t  fx;
    const fixture_def_t *def;
    geom_scene_t  scene;
    geo_origin_t  session;
    geo_origin_t  vehicle_origin[MAX_SIM_VEHICLES];
    double        vehicle_origin_lla[MAX_SIM_VEHICLES][3];
    int           vehicles;
    double        duration_s;
    double        rate_hz;
    emitter_t     em;
    truth_writer_t truth;
    bool          have_truth;
    uint64_t      ray_seq;
} sim_t;

// Session ENU -> this vehicle's local NED, via ECEF. The two-origins fixture
// exists to prove the viewer can undo exactly this.
static void enu_to_vehicle_ned(const sim_t *s, int veh, const double enu[3], double ned[3]) {
    double ecef[3], local[3];
    geo_enu_to_ecef(&s->session, enu, ecef);
    geo_ecef_to_enu(&s->vehicle_origin[veh], ecef, local);
    ned[0] = local[1];    // north
    ned[1] = local[0];    // east
    ned[2] = -local[2];   // down
}

static void emit_origin(sim_t *s, int veh, double t) {
    mavlink_message_t msg;
    const uint8_t sysid = (uint8_t)(veh + 1);
    mavlink_msg_gps_global_origin_pack(sysid, 1, &msg,
        (int32_t)lrint(s->vehicle_origin_lla[veh][0] * 1e7),
        (int32_t)lrint(s->vehicle_origin_lla[veh][1] * 1e7),
        (int32_t)lrint(s->vehicle_origin_lla[veh][2] * 1e3),
        (uint64_t)(t * 1e6));
    emit(&s->em, t, &msg);
}

static void emit_heartbeat(sim_t *s, int veh, double t) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack((uint8_t)(veh + 1), 1, &msg,
        MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
        MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4 /* custom mode */, MAV_STATE_ACTIVE);
    emit(&s->em, t, &msg);
}

static void emit_system_time(sim_t *s, int veh, double t) {
    mavlink_message_t msg;
    mavlink_msg_system_time_pack((uint8_t)(veh + 1), 1, &msg,
        (uint64_t)((BASE_UNIX_NS + (int64_t)(t * 1e9)) / 1000LL),
        (uint32_t)(t * 1000.0));
    emit(&s->em, t, &msg);
}

static void emit_gps_raw(sim_t *s, int veh, double t, const sim_pose_t *p) {
    double lat, lon, alt;
    const double enu[3] = {
        p->enu[0] + p->report_offset[0],
        p->enu[1] + p->report_offset[1],
        p->enu[2] + p->report_offset[2],
    };
    geo_enu_to_lla(&s->session, enu, &lat, &lon, &alt);

    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_pack((uint8_t)(veh + 1), 1, &msg,
        (uint64_t)(t * 1e6), GPS_FIX_TYPE_RTK_FIXED,
        (int32_t)lrint(lat * 1e7), (int32_t)lrint(lon * 1e7), (int32_t)lrint(alt * 1e3),
        80 /* eph = HDOP*100 = 0.80 */, 120 /* epv */, 0 /* vel */, UINT16_MAX /* cog */,
        18 /* sats */, (int32_t)lrint(alt * 1e3),
        35 /* h_acc mm */, 60 /* v_acc mm */, 40 /* vel_acc mm/s */, 0, 0);
    emit(&s->em, t, &msg);
}

static void emit_attitude(sim_t *s, int veh, double t, const double q[4]) {
    mavlink_message_t msg;
    mavlink_msg_attitude_quaternion_pack((uint8_t)(veh + 1), 1, &msg,
        (uint32_t)(t * 1000.0), (float)q[0], (float)q[1], (float)q[2], (float)q[3],
        0, 0, 0, NULL);
    emit(&s->em, t, &msg);
}

static void emit_position(sim_t *s, int veh, double t, const sim_pose_t *p) {
    const uint8_t sysid = (uint8_t)(veh + 1);
    mavlink_message_t msg;
    const double enu[3] = {
        p->enu[0] + p->report_offset[0],
        p->enu[1] + p->report_offset[1],
        p->enu[2] + p->report_offset[2],
    };
    double ned[3];
    enu_to_vehicle_ned(s, veh, enu, ned);
    mavlink_msg_local_position_ned_pack(sysid, 1, &msg, (uint32_t)(t * 1000.0),
        (float)ned[0], (float)ned[1], (float)ned[2], 0, 0, 0);
    emit(&s->em, t, &msg);
}

// The pose to publish for an `orientations` step: chosen so that whatever mount
// is under test, the sensor axis ends up pointing straight down. If the viewer's
// table disagrees with this one, the ray goes somewhere else and the surface RMS
// says so.
static void orientations_attitude(const sensor_cfg_t *c, double q_nb[4]) {
    double axis_body[3];
    if (c->use_quaternion) {
        const double qd[4] = { c->quaternion[0], c->quaternion[1],
                               c->quaternion[2], c->quaternion[3] };
        const double x[3] = { 1.0, 0.0, 0.0 };
        ob_quat_rotate(qd, x, axis_body);
    } else {
        ob_sensor_axis(c->orientation, axis_body);
    }
    const double down_ned[3] = { 0.0, 0.0, 1.0 };
    ob_quat_between(axis_body, down_ned, q_nb);
}

// The sensor's measurement axis in world ENU, given attitude and mount.
static void sensor_axis_enu(const sim_pose_t *p, const sensor_cfg_t *c,
                            const double *q_override, double out[3]) {
    double q_nb[4];
    if (q_override) memcpy(q_nb, q_override, sizeof(q_nb));
    else ob_quat_from_euler(p->roll, p->pitch, p->yaw, q_nb);

    double axis_body[3];
    if (c->use_quaternion) {
        const double qd[4] = { c->quaternion[0], c->quaternion[1],
                               c->quaternion[2], c->quaternion[3] };
        const double x[3] = { 1.0, 0.0, 0.0 };
        ob_quat_rotate(qd, x, axis_body);
    } else {
        ob_sensor_axis(c->orientation, axis_body);
    }

    double ned[3];
    ob_quat_rotate(q_nb, axis_body, ned);
    out[0] = ned[1];    // east
    out[1] = ned[0];    // north
    out[2] = -ned[2];   // up
}

static void record_truth_ray(sim_t *s, double t, int veh, const double origin[3],
                             const double dir[3], double range, bool hit) {
    if (!s->have_truth) return;
    const uint32_t stride = s->def->truth_ray_stride ? s->def->truth_ray_stride : 1;
    if ((s->ray_seq++ % stride) != 0) return;
    truth_ray_t r;
    memset(&r, 0, sizeof(r));
    r.t_s = t;
    for (int i = 0; i < 3; i++) {
        r.origin[i] = (float)origin[i];
        r.endpoint[i] = (float)(origin[i] + dir[i] * range);
    }
    r.vehicle = (uint8_t)veh;
    r.hit = hit ? 1 : 0;
    // The mount test walks one orientation at a time, so the step index is the
    // group the checker scores separately.
    if (s->fx == FX_ORIENTATIONS) {
        const int step = orientation_step(t);
        r.group = (uint8_t)((step >= 0 && step < 255) ? step : 255);
    }
    truth_writer_ray(&s->truth, &r);
}

static void emit_distance_sensor(sim_t *s, int veh, double t, const sim_pose_t *p) {
    sensor_cfg_t c;
    sensor_config(s->fx, veh, t, &c);

    double q_nb[4];
    const double *q_override = NULL;
    if (s->fx == FX_ORIENTATIONS) {
        orientations_attitude(&c, q_nb);
        q_override = q_nb;
    }

    double dir[3];
    sensor_axis_enu(p, &c, q_override, dir);

    const double max_m = (double)c.max_cm * 0.01;
    double range = max_m;
    bool hit = false;
    double t_hit;
    if (geom_scene_raycast(&s->scene, t, p->enu, dir, max_m, &t_hit, NULL)) {
        range = t_hit;
        hit = true;
    }
    record_truth_ray(s, t, veh, p->enu, dir, range, hit);

    // The mount test flies whatever attitude puts the sensor under test facing
    // down, so its attitude is published here rather than on the pose schedule.
    if (q_override) emit_attitude(s, veh, t, q_nb);

    // Range noise scaled by the declared covariance, so the `weak` fixture's
    // poor returns really are noisier and not merely labelled so.
    const double sigma_m = sqrt((double)c.covariance) * 0.01;
    double reported = range + rng_sym() * sigma_m;
    if (reported < 0.0) reported = 0.0;
    uint16_t cm = (uint16_t)lrint(reported * 100.0);
    if (!hit) cm = c.max_cm;   // no return reports exactly max range

    // The clock fixture back-dates vehicle 1's ranging stamps by 0.8 s. The
    // measurement itself is honest -- only the timestamp lies -- so the viewer
    // pairs a fresh range with an attitude from 0.8 s ago, which is precisely
    // the mistake that smears a flat wall into a curve.
    //
    // The skew is backwards rather than forwards on purpose: a forward-dated
    // stamp resolves to the newest pose available and quietly corrects itself,
    // so it would prove nothing.
    double stamp_t = t;
    if (s->fx == FX_CLOCKS && veh == 1) {
        stamp_t = t - 0.8;
        if (stamp_t < 0.0) stamp_t = 0.0;
    }

    mavlink_message_t msg;
    mavlink_msg_distance_sensor_pack((uint8_t)(veh + 1), 1, &msg,
        (uint32_t)(stamp_t * 1000.0), c.min_cm, c.max_cm, cm,
        MAV_DISTANCE_SENSOR_LASER, 0 /* id */, c.orientation, c.covariance,
        c.h_fov, c.v_fov, c.use_quaternion ? c.quaternion : NULL, c.signal_quality);
    emit(&s->em, t, &msg);

    // A downward-looking fixture should also say how high it thinks it is; the
    // `ground` check compares that against the map.
    if (s->fx == FX_GROUND) {
        mavlink_message_t alt;
        mavlink_msg_altitude_pack((uint8_t)(veh + 1), 1, &alt, (uint64_t)(t * 1e6),
            (float)p->enu[2], (float)(SESSION_ALT + p->enu[2]), (float)p->enu[2],
            (float)p->enu[2], 0.0f, hit ? (float)range : NAN);
        emit(&s->em, t, &alt);

        double lat, lon, altm;
        geo_enu_to_lla(&s->session, p->enu, &lat, &lon, &altm);
        mavlink_message_t gp;
        mavlink_msg_global_position_int_pack((uint8_t)(veh + 1), 1, &gp,
            (uint32_t)(t * 1000.0), (int32_t)lrint(lat * 1e7), (int32_t)lrint(lon * 1e7),
            (int32_t)lrint(altm * 1e3), (int32_t)lrint(p->enu[2] * 1e3), 0, 0, 0, 0);
        emit(&s->em, t, &gp);
    }
}

static void emit_obstacle_distance(sim_t *s, int veh, double t, const sim_pose_t *p) {
    const int sectors = OBSTACLE_DISTANCE_SECTORS;
    const float increment = 360.0f / (float)sectors;
    const uint16_t min_cm = 20, max_cm = 3000;
    const double max_m = (double)max_cm * 0.01;

    uint16_t distances[OBSTACLE_DISTANCE_SECTORS];
    double q_nb[4];
    ob_quat_from_euler(p->roll, p->pitch, p->yaw, q_nb);

    for (int i = 0; i < sectors; i++) {
        // Sector i is `angle_offset + i * increment` from the vehicle nose,
        // measured in body FRD. Off-by-one here rotates the whole map.
        const double bearing = (double)i * (double)increment * DEG;
        const double body[3] = { cos(bearing), sin(bearing), 0.0 };
        double ned[3];
        ob_quat_rotate(q_nb, body, ned);
        const double dir[3] = { ned[1], ned[0], -ned[2] };

        double range = max_m;
        bool hit = false;
        double t_hit;
        if (geom_scene_raycast(&s->scene, t, p->enu, dir, max_m, &t_hit, NULL)) {
            range = t_hit;
            hit = true;
        }
        record_truth_ray(s, t, veh, p->enu, dir, range, hit);
        distances[i] = (uint16_t)lrint(range * 100.0);
    }

    mavlink_message_t msg;
    mavlink_msg_obstacle_distance_pack((uint8_t)(veh + 1), 1, &msg,
        (uint64_t)(t * 1e6), MAV_DISTANCE_SENSOR_LASER, distances,
        (uint8_t)lrint(increment), min_cm, max_cm, increment, 0.0f, MAV_FRAME_BODY_FRD);
    emit(&s->em, t, &msg);
}

// ------------------------------------------------------------ main loop

static void sleep_until(double sim_t, double start_wall) {
#ifndef _WIN32
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    const double now = (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
    const double want = start_wall + sim_t;
    if (want > now) {
        struct timespec req;
        const double d = want - now;
        req.tv_sec = (time_t)d;
        req.tv_nsec = (long)((d - (double)req.tv_sec) * 1e9);
        nanosleep(&req, NULL);
    }
#else
    (void)sim_t; (void)start_wall;
#endif
}

static double wall_now(void) {
#ifndef _WIN32
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#else
    return 0.0;
#endif
}

static int run(sim_t *s, uint32_t seed, double scale) {
    rng_seed(seed);

    const double dt = 1.0 / s->rate_hz;
    const int ticks = (int)(s->duration_s / dt);
    const int hb_every = (int)(s->rate_hz);            // 1 Hz
    const double start_wall = wall_now();

    for (int veh = 0; veh < s->vehicles; veh++) emit_origin(s, veh, 0.0);

    for (int k = 0; k < ticks; k++) {
        const double t = (double)k * dt;
        for (int veh = 0; veh < s->vehicles; veh++) {
            sim_pose_t p;
            vehicle_pose(s->fx, veh, t, &p);

            if (hb_every > 0 && (k % hb_every) == 0) {
                emit_heartbeat(s, veh, t);
                emit_system_time(s, veh, t);
                emit_gps_raw(s, veh, t, &p);
                if ((k % (hb_every * 5)) == 0) emit_origin(s, veh, t);
            }

            // The `moving` fixture publishes pose at half the ranging rate on
            // purpose: pairing a fresh range with a stale attitude smears a flat
            // wall into a curve, and this is where that shows up.
            const int divisor = s->def->pose_rate_divisor > 0 ? (int)s->def->pose_rate_divisor : 1;
            if ((k % divisor) == 0) {
                emit_position(s, veh, t, &p);
                if (s->fx != FX_ORIENTATIONS) {
                    double q[4];
                    ob_quat_from_euler(p.roll, p.pitch, p.yaw, q);
                    emit_attitude(s, veh, t, q);
                }
            }

            if (s->def->sensor == SENSOR_OBSTACLE) emit_obstacle_distance(s, veh, t, &p);
            else                                    emit_distance_sensor(s, veh, t, &p);
        }
        if (s->em.realtime) sleep_until(t, start_wall);
    }

    if (!s->have_truth) return 0;

    truth_header_t h;
    memset(&h, 0, sizeof(h));
    snprintf(h.fixture, sizeof(h.fixture), "%s", s->def->name);
    h.seed = seed;
    h.duration_s = s->duration_s;
    h.scale = scale;
    h.session_lat = SESSION_LAT;
    h.session_lon = SESSION_LON;
    h.session_alt = SESSION_ALT;
    h.vehicle_count = s->vehicles;
    for (int veh = 0; veh < s->vehicles; veh++) {
        h.vehicles[veh].sysid = (uint8_t)(veh + 1);
        h.vehicles[veh].origin_lat = s->vehicle_origin_lla[veh][0];
        h.vehicles[veh].origin_lon = s->vehicle_origin_lla[veh][1];
        h.vehicles[veh].origin_alt = s->vehicle_origin_lla[veh][2];
        sim_pose_t p;
        vehicle_pose(s->fx, veh, 0.0, &p);
        memcpy(h.vehicles[veh].pos_offset_enu, p.report_offset, sizeof(p.report_offset));
    }
    h.scene = s->scene;
    h.root_size_m = 1024.0;
    h.max_depth = 12;          // 0.25 m leaves under a 1024 m root
    h.coarse_depth = 9;        // 2 m far-field carving
    h.skip_near_m = 1.0;
    h.queue_capacity = s->def->queue_capacity;
    h.budget_per_drain = s->def->budget_per_drain;
    h.map_byte_cap = (size_t)512 * 1024 * 1024;
    h.thresholds = s->def->th;

    // The load-dependent thresholds have to follow the load. `firehose` is the
    // one fixture whose vehicle count is a knob, and it is documented as a way
    // to trim CI cost -- but its throughput floor and its "drops must happen"
    // assertion are properties of eight vehicles. Turning the knob down used to
    // fail the fixture for doing exactly what it was asked.
    if (s->def->vehicles > 0 && s->vehicles != s->def->vehicles) {
        const double load = (double)s->vehicles / (double)s->def->vehicles;
        h.thresholds.min_rays_per_s *= load;
        h.thresholds.occupied_cells_min =
            (int64_t)((double)h.thresholds.occupied_cells_min * load);
        // Below the nominal fleet the queue may never overrun, and asserting a
        // drop would be asserting something this run does not produce.
        if (load < 1.0) h.thresholds.require_drops = 0;
    }

    return truth_writer_finish(&s->truth, &h);
}

static void usage(void) {
    printf("ray_injector --fixture <name> [options]\n");
    printf("  --fixture <name>   one of:");
    for (int i = 0; i < FX_COUNT; i++) printf(" %s", k_fixtures[i].name);
    printf("\n");
    printf("  --tlog <path>      record the emitted frames\n");
    printf("  --truth <prefix>   publish ground truth as <prefix>.json / <prefix>.rays\n");
    printf("  --udp <host:port>  also emit over the wire\n");
    printf("  --realtime         pace UDP emission against the wall clock\n");
    printf("  --seed <n>         RNG seed (default 1)\n");
    printf("  --scale <f>        scale vehicle count and duration (default 1)\n");
    printf("  --list             print fixture names, one per line\n");
}

int main(int argc, char **argv) {
    const char *fixture_name = NULL;
    const char *tlog_path = NULL;
    const char *truth_prefix = NULL;
    const char *udp_target = NULL;
    uint32_t seed = 1;
    double scale = 1.0;
    bool realtime = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--fixture") == 0 && i + 1 < argc) fixture_name = argv[++i];
        else if (strcmp(argv[i], "--tlog") == 0 && i + 1 < argc) tlog_path = argv[++i];
        else if (strcmp(argv[i], "--truth") == 0 && i + 1 < argc) truth_prefix = argv[++i];
        else if (strcmp(argv[i], "--udp") == 0 && i + 1 < argc) udp_target = argv[++i];
        else if (strcmp(argv[i], "--realtime") == 0) realtime = true;
        else if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) seed = (uint32_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "--scale") == 0 && i + 1 < argc) scale = atof(argv[++i]);
        else if (strcmp(argv[i], "--list") == 0) {
            for (int k = 0; k < FX_COUNT; k++) printf("%s\n", k_fixtures[k].name);
            return 0;
        } else { usage(); return 2; }
    }
    if (!fixture_name) { usage(); return 2; }

    fixture_id_t fx = FX_COUNT;
    for (int i = 0; i < FX_COUNT; i++)
        if (strcmp(k_fixtures[i].name, fixture_name) == 0) { fx = (fixture_id_t)i; break; }
    if (fx == FX_COUNT) {
        fprintf(stderr, "unknown fixture '%s'\n", fixture_name);
        return 2;
    }

    sim_t s;
    memset(&s, 0, sizeof(s));
    s.fx = fx;
    s.def = &k_fixtures[fx];
    s.duration_s = s.def->duration_s * (scale > 0.0 ? scale : 1.0);
    s.rate_hz = s.def->rate_hz;
    s.vehicles = s.def->vehicles;
    if (scale > 0.0 && (fx == FX_FIREHOSE)) {
        s.vehicles = (int)lrint((double)s.def->vehicles * scale);
        s.duration_s = s.def->duration_s;
    }
    if (s.vehicles < 1) s.vehicles = 1;
    if (s.vehicles > MAX_SIM_VEHICLES) s.vehicles = MAX_SIM_VEHICLES;

    build_scene(fx, &s.scene);
    geo_origin_set(&s.session, SESSION_LAT, SESSION_LON, SESSION_ALT);

    for (int veh = 0; veh < s.vehicles; veh++) {
        double lat = SESSION_LAT, lon = SESSION_LON, alt = SESSION_ALT;
        if (fx == FX_TWO_ORIGINS && veh == 1) {
            // Deliberately a different origin, ~900 m east and 12 m higher. Both
            // vehicles must still land on one surface.
            const double enu[3] = { 900.0, 0.0, 12.0 };
            geo_enu_to_lla(&s.session, enu, &lat, &lon, &alt);
        } else if (veh > 0) {
            // A small but non-zero offset for every other multi-vehicle fixture,
            // so the ECEF path is exercised rather than short-circuited.
            const double enu[3] = { 25.0 * veh, -15.0 * veh, 3.0 };
            geo_enu_to_lla(&s.session, enu, &lat, &lon, &alt);
        }
        s.vehicle_origin_lla[veh][0] = lat;
        s.vehicle_origin_lla[veh][1] = lon;
        s.vehicle_origin_lla[veh][2] = alt;
        geo_origin_set(&s.vehicle_origin[veh], lat, lon, alt);
    }

    if (tlog_path) {
        if (tlog_writer_open(&s.em.tlog, tlog_path) != 0) {
            fprintf(stderr, "cannot write tlog %s\n", tlog_path);
            return 1;
        }
        s.em.have_tlog = true;
    }
    if (truth_prefix) {
        if (truth_writer_open(&s.truth, truth_prefix) != 0) {
            fprintf(stderr, "cannot write truth %s\n", truth_prefix);
            return 1;
        }
        s.have_truth = true;
    }
    if (udp_target) {
#ifdef _WIN32
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
        char host[128];
        int port = 0;
        const char *colon = strrchr(udp_target, ':');
        if (!colon) { fprintf(stderr, "--udp wants host:port\n"); return 2; }
        const size_t hl = (size_t)(colon - udp_target);
        if (hl >= sizeof(host)) { fprintf(stderr, "--udp host too long\n"); return 2; }
        memcpy(host, udp_target, hl);
        host[hl] = '\0';
        port = atoi(colon + 1);

        s.em.sock = (int)socket(AF_INET, SOCK_DGRAM, 0);
        if (s.em.sock < 0) { perror("socket"); return 1; }
        memset(&s.em.dest, 0, sizeof(s.em.dest));
        s.em.dest.sin_family = AF_INET;
        s.em.dest.sin_port = htons((uint16_t)port);
        if (inet_pton(AF_INET, host, &s.em.dest.sin_addr) != 1) {
            fprintf(stderr, "bad --udp host '%s'\n", host);
            return 2;
        }
        s.em.have_udp = true;
        s.em.realtime = realtime;
    }

    const int rc = run(&s, seed, scale);

    if (s.em.have_tlog) tlog_writer_close(&s.em.tlog);
    printf("fixture=%s vehicles=%d duration=%.1fs messages=%llu rays=%u\n",
           s.def->name, s.vehicles, s.duration_s,
           (unsigned long long)s.em.messages, s.truth.ray_count);
    return rc;
}
