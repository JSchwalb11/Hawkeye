#ifndef RAY_TRANSFORM_H
#define RAY_TRANSFORM_H

// Ranging observations -> world-frame rays.
//
//   p_world = origin_vehicle
//           + R_ned<-body(attitude)
//           . R_body<-sensor(orientation | quaternion)
//           . [distance, 0, 0]
//
// This layer is deliberately free of MAVLink types so the ULog and DataFlash
// paths can feed it the same way the live path does. It is also where three
// easy-to-get-wrong rules live:
//
//  * A reading at max_distance is a *no return*, not a hit. Treating it as a
//    hit builds a wall at sensor range around every flight.
//  * horizontal_fov / vertical_fov make the ray a cone. The endpoint widens
//    with distance; a 25-degree sonar is not a laser.
//  * covariance and signal_quality scale the evidence. A weak return should
//    move the map less than a clean one.

#include <stdbool.h>
#include <stdint.h>

#include "octomap.h"

#ifdef __cplusplus
extern "C" {
#endif

// MAV_SENSOR_ROTATION_NONE (0) through MAV_SENSOR_ROTATION_ROLL_90_PITCH_315
// (40) are contiguous; the enum then jumps to MAV_SENSOR_ROTATION_CUSTOM=100,
// which only means anything alongside a quaternion.
#define MAV_SENSOR_ORIENTATION_COUNT 41
#define OBSTACLE_DISTANCE_SECTORS    72

// --- Small quaternion helpers (w, x, y, z) -----------------------------

void rt_quat_from_euler(float roll_rad, float pitch_rad, float yaw_rad, float q[4]);
void rt_quat_mul(const float a[4], const float b[4], float out[4]);
void rt_quat_rotate(const float q[4], const double v[3], double out[3]);
void rt_quat_normalize(float q[4]);

// R_body<-sensor for a MAV_SENSOR_ORIENTATION value. Returns false for values
// outside the enum, in which case `q` is set to identity.
bool rt_sensor_orientation_quat(uint8_t orientation, float q[4]);

// Human-readable name, e.g. "PITCH_270". Never NULL.
const char *rt_sensor_orientation_name(uint8_t orientation);

// NED vector -> ENU vector.
void rt_ned_to_enu(const double ned[3], double enu[3]);

// --- Single ranging observation ----------------------------------------

typedef struct {
    double   origin_enu[3];      // sensor origin, session ENU metres
    float    att_ned_body[4];    // R_ned<-body as a quaternion
    uint8_t  orientation;        // MAV_SENSOR_ORIENTATION
    bool     have_quaternion;    // when set, sensor_q wins over `orientation`
    float    sensor_q[4];        // R_body<-sensor override
    float    distance_m;
    float    min_distance_m;
    float    max_distance_m;
    float    horizontal_fov_rad; // 0 = unknown
    float    vertical_fov_rad;   // 0 = unknown
    uint8_t  covariance_cm2;     // 255 = unknown
    uint8_t  signal_quality;     // 0 = unknown, else 1..100
    uint8_t  vehicle_id;
    uint32_t time_ms;
} ray_obs_t;

// Build the map-ready ray. Returns false when the observation carries no usable
// information (below min range, non-finite, zero-length).
bool rt_build_ray(const ray_obs_t *obs, om_ray_t *out);

// Evidence weight in [0,1] from covariance and signal quality. Exposed so the
// `weak` fixture can assert the weighting directly.
float rt_evidence_weight(uint8_t covariance_cm2, uint8_t signal_quality);

// Endpoint half-width in metres for a cone of the given FOV at that range.
float rt_cone_radius(float distance_m, float h_fov_rad, float v_fov_rad);

// --- OBSTACLE_DISTANCE -------------------------------------------------

// Frame values we care about; matching MAV_FRAME.
#define RT_FRAME_GLOBAL     0
#define RT_FRAME_BODY_FRD  12

typedef struct {
    double   origin_enu[3];
    float    att_ned_body[4];
    uint16_t distances_cm[OBSTACLE_DISTANCE_SECTORS];
    uint8_t  sector_count;       // usually 72
    uint8_t  frame;              // RT_FRAME_GLOBAL or RT_FRAME_BODY_FRD
    float    increment_deg;      // resolved: increment_f when non-zero, else increment
    float    angle_offset_deg;
    uint16_t min_distance_cm;
    uint16_t max_distance_cm;
    uint8_t  vehicle_id;
    uint32_t time_ms;
} obstacle_obs_t;

// Expand the sectors into rays sharing one origin and pose. Returns the count
// written, never more than `max_out`.
int rt_expand_obstacle_distance(const obstacle_obs_t *obs, om_ray_t *out, int max_out);

#ifdef __cplusplus
}
#endif

#endif
