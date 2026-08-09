#include "ray_transform.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RT_DEG2RAD ((float)(M_PI / 180.0))

// ------------------------------------------------------------ quaternions

void rt_quat_from_euler(float roll, float pitch, float yaw, float q[4]) {
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

void rt_quat_mul(const float a[4], const float b[4], float out[4]) {
    const float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    const float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    const float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    const float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void rt_quat_normalize(float q[4]) {
    const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 1e-9f) { q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n; }
    else { q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f; }
}

void rt_quat_rotate(const float q[4], const double v[3], double out[3]) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    // v' = v + 2 * cross(qv, cross(qv, v) + w*v)
    const double tx = 2.0 * (y * v[2] - z * v[1]);
    const double ty = 2.0 * (z * v[0] - x * v[2]);
    const double tz = 2.0 * (x * v[1] - y * v[0]);
    out[0] = v[0] + w * tx + (y * tz - z * ty);
    out[1] = v[1] + w * ty + (z * tx - x * tz);
    out[2] = v[2] + w * tz + (x * ty - y * tx);
}

void rt_ned_to_enu(const double ned[3], double enu[3]) {
    enu[0] = ned[1];    // east
    enu[1] = ned[0];    // north
    enu[2] = -ned[2];   // up
}

// -------------------------------------------------- sensor orientation table

// MAV_SENSOR_ORIENTATION as (roll, pitch, yaw) degrees, applied in the standard
// aerospace order R = Rz(yaw) . Ry(pitch) . Rx(roll). Every one of these is a
// chance to be wrong, which is why the `orientations` fixture sweeps all of
// them against a single plane.
typedef struct { float roll, pitch, yaw; const char *name; } rt_orient_t;

static const rt_orient_t k_orientations[MAV_SENSOR_ORIENTATION_COUNT] = {
    {   0,   0,   0.0f, "NONE" },
    {   0,   0,  45.0f, "YAW_45" },
    {   0,   0,  90.0f, "YAW_90" },
    {   0,   0, 135.0f, "YAW_135" },
    {   0,   0, 180.0f, "YAW_180" },
    {   0,   0, 225.0f, "YAW_225" },
    {   0,   0, 270.0f, "YAW_270" },
    {   0,   0, 315.0f, "YAW_315" },
    { 180,   0,   0.0f, "ROLL_180" },
    { 180,   0,  45.0f, "ROLL_180_YAW_45" },
    { 180,   0,  90.0f, "ROLL_180_YAW_90" },
    { 180,   0, 135.0f, "ROLL_180_YAW_135" },
    {   0, 180,   0.0f, "PITCH_180" },
    { 180,   0, 225.0f, "ROLL_180_YAW_225" },
    { 180,   0, 270.0f, "ROLL_180_YAW_270" },
    { 180,   0, 315.0f, "ROLL_180_YAW_315" },
    {  90,   0,   0.0f, "ROLL_90" },
    {  90,   0,  45.0f, "ROLL_90_YAW_45" },
    {  90,   0,  90.0f, "ROLL_90_YAW_90" },
    {  90,   0, 135.0f, "ROLL_90_YAW_135" },
    { 270,   0,   0.0f, "ROLL_270" },
    { 270,   0,  45.0f, "ROLL_270_YAW_45" },
    { 270,   0,  90.0f, "ROLL_270_YAW_90" },
    { 270,   0, 135.0f, "ROLL_270_YAW_135" },
    {   0,  90,   0.0f, "PITCH_90" },
    {   0, 270,   0.0f, "PITCH_270" },
    {   0, 180,  90.0f, "PITCH_180_YAW_90" },
    {   0, 180, 270.0f, "PITCH_180_YAW_270" },
    {  90,  90,   0.0f, "ROLL_90_PITCH_90" },
    { 180,  90,   0.0f, "ROLL_180_PITCH_90" },
    { 270,  90,   0.0f, "ROLL_270_PITCH_90" },
    {  90, 180,   0.0f, "ROLL_90_PITCH_180" },
    { 270, 180,   0.0f, "ROLL_270_PITCH_180" },
    {  90, 270,   0.0f, "ROLL_90_PITCH_270" },
    { 180, 270,   0.0f, "ROLL_180_PITCH_270" },
    { 270, 270,   0.0f, "ROLL_270_PITCH_270" },
    {  90, 180,  90.0f, "ROLL_90_PITCH_180_YAW_90" },
    {  90,   0, 270.0f, "ROLL_90_YAW_270" },
    // The enum's own comment rounds this one to whole degrees, but the mount it
    // names is ArduPilot's, and ArduPilot builds it from 68.8 and 293.3. At 30 m
    // the rounding is 0.4 m of boresight error for no reason.
    {  90,  68.8f, 293.3f, "ROLL_90_PITCH_68_YAW_293" },
    {   0, 315,   0.0f, "PITCH_315" },
    {  90, 315,   0.0f, "ROLL_90_PITCH_315" },
};

bool rt_sensor_orientation_quat(uint8_t orientation, float q[4]) {
    if (orientation >= MAV_SENSOR_ORIENTATION_COUNT) {
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
        return false;
    }
    const rt_orient_t *o = &k_orientations[orientation];
    rt_quat_from_euler(o->roll * RT_DEG2RAD, o->pitch * RT_DEG2RAD,
                       o->yaw * RT_DEG2RAD, q);
    return true;
}

const char *rt_sensor_orientation_name(uint8_t orientation) {
    if (orientation >= MAV_SENSOR_ORIENTATION_COUNT) return "CUSTOM";
    return k_orientations[orientation].name;
}

// ------------------------------------------------------------ weighting

float rt_evidence_weight(uint8_t covariance_cm2, uint8_t signal_quality) {
    float w = 1.0f;

    // signal_quality: 0 means "unknown", 1..100 is a percentage.
    if (signal_quality > 0) {
        float q = (float)signal_quality;
        if (q > 100.0f) q = 100.0f;
        w *= q / 100.0f;
    }

    // covariance is the measurement variance in cm^2; 255 means unknown.
    if (covariance_cm2 != 255) {
        const float sigma_cm = sqrtf((float)covariance_cm2);
        const float ref_cm = 3.0f;
        const float r = sigma_cm / ref_cm;
        w *= 1.0f / (1.0f + r * r);
    }

    if (w < 0.0f) w = 0.0f;
    if (w > 1.0f) w = 1.0f;
    return w;
}

float rt_cone_radius(float distance_m, float h_fov_rad, float v_fov_rad) {
    float fov = h_fov_rad > v_fov_rad ? h_fov_rad : v_fov_rad;
    if (!(fov > 0.0f) || !isfinite(fov)) return 0.0f;
    if (fov > (float)M_PI * 0.9f) fov = (float)M_PI * 0.9f;
    return distance_m * tanf(fov * 0.5f);
}

// ------------------------------------------------------------ ray building

bool rt_build_ray(const ray_obs_t *obs, om_ray_t *out) {
    if (!obs || !out) return false;
    if (!isfinite(obs->distance_m)) return false;

    const float min_d = obs->min_distance_m;
    const bool have_max = (obs->max_distance_m > 0.0f);
    const float max_d = have_max ? obs->max_distance_m : obs->distance_m;

    // Below min range the sensor is telling us about itself, not the world.
    if (obs->distance_m < min_d) return false;
    if (max_d <= 0.0f) return false;

    // At (or beyond) max range there is no return. Carve out to max range and
    // mark nothing occupied -- this is the difference between a map and a
    // cylinder of phantom walls at sensor range.
    //
    // A sender that leaves max_distance unset is saying it does not know its
    // own range, not that every reading is at it. Substituting the reading for
    // the ceiling and then comparing the two would turn every such return into
    // a no-return and the map would never hold a surface at all.
    bool hit = true;
    float range = obs->distance_m;
    if (have_max && range >= max_d) { hit = false; range = max_d; }

    float q_bs[4];
    if (obs->have_quaternion) {
        memcpy(q_bs, obs->sensor_q, sizeof(q_bs));
        rt_quat_normalize(q_bs);
    } else if (!rt_sensor_orientation_quat(obs->orientation, q_bs)) {
        // An orientation this build does not know is not an excuse to aim the
        // ray down body +X. A mount we cannot place produces a phantom
        // obstacle dead ahead and carves free space through wherever the
        // sensor was really pointing, both silently. Reject it instead.
        return false;
    }

    float q_nb[4];
    memcpy(q_nb, obs->att_ned_body, sizeof(q_nb));
    rt_quat_normalize(q_nb);

    float q_ns[4];
    rt_quat_mul(q_nb, q_bs, q_ns);

    const double axis[3] = { (double)range, 0.0, 0.0 };   // sensor +X
    double ned[3], enu[3];
    rt_quat_rotate(q_ns, axis, ned);
    rt_ned_to_enu(ned, enu);

    memset(out, 0, sizeof(*out));
    out->origin[0] = obs->origin_enu[0];
    out->origin[1] = obs->origin_enu[1];
    out->origin[2] = obs->origin_enu[2];
    out->endpoint[0] = obs->origin_enu[0] + enu[0];
    out->endpoint[1] = obs->origin_enu[1] + enu[1];
    out->endpoint[2] = obs->origin_enu[2] + enu[2];
    out->hit = hit;
    out->weight = rt_evidence_weight(obs->covariance_cm2, obs->signal_quality);
    // A no-return sweeps its whole cone clear, not just the axis, so the width
    // is as load-bearing here as it is on a hit -- zeroing it would carve a
    // pencil through a volume the sensor cleared to its full beam width.
    out->cone_radius_m = rt_cone_radius(range, obs->horizontal_fov_rad,
                                        obs->vertical_fov_rad);
    out->vehicle_id = obs->vehicle_id;
    out->time_ms = obs->time_ms;
    return true;
}

// --------------------------------------------------- OBSTACLE_DISTANCE

int rt_expand_obstacle_distance(const obstacle_obs_t *obs, om_ray_t *out, int max_out) {
    if (!obs || !out || max_out <= 0) return 0;

    int sectors = obs->sector_count ? obs->sector_count : OBSTACLE_DISTANCE_SECTORS;
    if (sectors > OBSTACLE_DISTANCE_SECTORS) sectors = OBSTACLE_DISTANCE_SECTORS;

    // A signed increment carries the sweep direction: negative means the fan
    // runs counter-clockwise from angle_offset. Only an exactly-zero (or
    // non-finite) increment means "unspecified", and only then do we assume
    // the sectors evenly divide a full turn clockwise.
    float increment = obs->increment_deg;
    if (increment == 0.0f || !isfinite(increment)) increment = 360.0f / (float)sectors;

    const float min_m = (float)obs->min_distance_cm * 0.01f;
    const float max_m = (float)obs->max_distance_cm * 0.01f;
    if (!(max_m > 0.0f)) return 0;

    float q_nb[4];
    memcpy(q_nb, obs->att_ned_body, sizeof(q_nb));
    rt_quat_normalize(q_nb);

    int n = 0;
    for (int i = 0; i < sectors && n < max_out; i++) {
        const uint16_t raw = obs->distances_cm[i];
        if (raw == UINT16_MAX) continue;   // no measurement at all for this sector

        float range = (float)raw * 0.01f;
        bool hit = true;
        if (range < min_m) continue;
        if (range >= max_m) { hit = false; range = max_m; }

        const float bearing = (obs->angle_offset_deg + increment * (float)i) * RT_DEG2RAD;
        const double dir_local[3] = { cos(bearing) * range, sin(bearing) * range, 0.0 };

        double ned[3];
        if (obs->frame == RT_FRAME_BODY_FRD) {
            // Sector angles are relative to the vehicle nose, so rotate by attitude.
            rt_quat_rotate(q_nb, dir_local, ned);
        } else {
            // MAV_FRAME_GLOBAL: index 0 is north + angle_offset, already NED.
            ned[0] = dir_local[0]; ned[1] = dir_local[1]; ned[2] = dir_local[2];
        }

        double enu[3];
        rt_ned_to_enu(ned, enu);

        om_ray_t *r = &out[n++];
        memset(r, 0, sizeof(*r));
        memcpy(r->origin, obs->origin_enu, sizeof(r->origin));
        r->endpoint[0] = obs->origin_enu[0] + enu[0];
        r->endpoint[1] = obs->origin_enu[1] + enu[1];
        r->endpoint[2] = obs->origin_enu[2] + enu[2];
        r->hit = hit;
        r->weight = 1.0f;
        // A sector spans `increment` degrees of azimuth; treat that as the cone.
        // Set on a no-return too: that sector is clear to its full width.
        r->cone_radius_m = rt_cone_radius(range, increment * RT_DEG2RAD, 0.0f);
        r->vehicle_id = obs->vehicle_id;
        r->time_ms = obs->time_ms;
    }
    return n;
}
