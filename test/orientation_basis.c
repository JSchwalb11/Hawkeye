#include "orientation_basis.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Transcribed from the MAV_SENSOR_ORIENTATION names, independently of
// src/ray_transform.c. Order is ROLL, PITCH, YAW in degrees; the rotation is
// applied as Rz(yaw) . Ry(pitch) . Rx(roll).
typedef struct { double roll, pitch, yaw; const char *name; } ob_entry_t;

static const ob_entry_t k_table[OB_ORIENTATION_COUNT] = {
    /*  0 */ {   0,   0,   0,   "ROTATION_NONE" },
    /*  1 */ {   0,   0,  45,   "ROTATION_YAW_45" },
    /*  2 */ {   0,   0,  90,   "ROTATION_YAW_90" },
    /*  3 */ {   0,   0, 135,   "ROTATION_YAW_135" },
    /*  4 */ {   0,   0, 180,   "ROTATION_YAW_180" },
    /*  5 */ {   0,   0, 225,   "ROTATION_YAW_225" },
    /*  6 */ {   0,   0, 270,   "ROTATION_YAW_270" },
    /*  7 */ {   0,   0, 315,   "ROTATION_YAW_315" },
    /*  8 */ { 180,   0,   0,   "ROTATION_ROLL_180" },
    /*  9 */ { 180,   0,  45,   "ROTATION_ROLL_180_YAW_45" },
    /* 10 */ { 180,   0,  90,   "ROTATION_ROLL_180_YAW_90" },
    /* 11 */ { 180,   0, 135,   "ROTATION_ROLL_180_YAW_135" },
    /* 12 */ {   0, 180,   0,   "ROTATION_PITCH_180" },
    /* 13 */ { 180,   0, 225,   "ROTATION_ROLL_180_YAW_225" },
    /* 14 */ { 180,   0, 270,   "ROTATION_ROLL_180_YAW_270" },
    /* 15 */ { 180,   0, 315,   "ROTATION_ROLL_180_YAW_315" },
    /* 16 */ {  90,   0,   0,   "ROTATION_ROLL_90" },
    /* 17 */ {  90,   0,  45,   "ROTATION_ROLL_90_YAW_45" },
    /* 18 */ {  90,   0,  90,   "ROTATION_ROLL_90_YAW_90" },
    /* 19 */ {  90,   0, 135,   "ROTATION_ROLL_90_YAW_135" },
    /* 20 */ { 270,   0,   0,   "ROTATION_ROLL_270" },
    /* 21 */ { 270,   0,  45,   "ROTATION_ROLL_270_YAW_45" },
    /* 22 */ { 270,   0,  90,   "ROTATION_ROLL_270_YAW_90" },
    /* 23 */ { 270,   0, 135,   "ROTATION_ROLL_270_YAW_135" },
    /* 24 */ {   0,  90,   0,   "ROTATION_PITCH_90" },
    /* 25 */ {   0, 270,   0,   "ROTATION_PITCH_270" },
    /* 26 */ {   0, 180,  90,   "ROTATION_PITCH_180_YAW_90" },
    /* 27 */ {   0, 180, 270,   "ROTATION_PITCH_180_YAW_270" },
    /* 28 */ {  90,  90,   0,   "ROTATION_ROLL_90_PITCH_90" },
    /* 29 */ { 180,  90,   0,   "ROTATION_ROLL_180_PITCH_90" },
    /* 30 */ { 270,  90,   0,   "ROTATION_ROLL_270_PITCH_90" },
    /* 31 */ {  90, 180,   0,   "ROTATION_ROLL_90_PITCH_180" },
    /* 32 */ { 270, 180,   0,   "ROTATION_ROLL_270_PITCH_180" },
    /* 33 */ {  90, 270,   0,   "ROTATION_ROLL_90_PITCH_270" },
    /* 34 */ { 180, 270,   0,   "ROTATION_ROLL_180_PITCH_270" },
    /* 35 */ { 270, 270,   0,   "ROTATION_ROLL_270_PITCH_270" },
    /* 36 */ {  90, 180,  90,   "ROTATION_ROLL_90_PITCH_180_YAW_90" },
    /* 37 */ {  90,   0, 270,   "ROTATION_ROLL_90_YAW_270" },
    /* 38 */ {  90,  68, 293,   "ROTATION_ROLL_90_PITCH_68_YAW_293" },
    /* 39 */ {   0, 315,   0,   "ROTATION_PITCH_315" },
    /* 40 */ {  90, 315,   0,   "ROTATION_ROLL_90_PITCH_315" },
};

static void mat_mul(const double a[9], const double b[9], double out[9]) {
    double r[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r[i * 3 + j] = a[i * 3 + 0] * b[0 * 3 + j]
                         + a[i * 3 + 1] * b[1 * 3 + j]
                         + a[i * 3 + 2] * b[2 * 3 + j];
    memcpy(out, r, sizeof(r));
}

bool ob_rotation(uint8_t orientation, double m[9]) {
    static const double identity[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    memcpy(m, identity, sizeof(identity));
    if (orientation >= OB_ORIENTATION_COUNT) return false;

    const double r = k_table[orientation].roll  * M_PI / 180.0;
    const double p = k_table[orientation].pitch * M_PI / 180.0;
    const double y = k_table[orientation].yaw   * M_PI / 180.0;

    const double rx[9] = { 1, 0, 0,  0, cos(r), -sin(r),  0, sin(r), cos(r) };
    const double ry[9] = { cos(p), 0, sin(p),  0, 1, 0,  -sin(p), 0, cos(p) };
    const double rz[9] = { cos(y), -sin(y), 0,  sin(y), cos(y), 0,  0, 0, 1 };

    double zy[9];
    mat_mul(rz, ry, zy);
    mat_mul(zy, rx, m);
    return true;
}

bool ob_sensor_axis(uint8_t orientation, double axis[3]) {
    double m[9];
    const bool ok = ob_rotation(orientation, m);
    axis[0] = m[0];   // first column
    axis[1] = m[3];
    axis[2] = m[6];
    return ok;
}

const char *ob_name(uint8_t orientation) {
    if (orientation >= OB_ORIENTATION_COUNT) return "ROTATION_CUSTOM";
    return k_table[orientation].name;
}

void ob_quat_mul(const double a[4], const double b[4], double out[4]) {
    const double w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    const double x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    const double y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    const double z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void ob_quat_from_euler(double roll, double pitch, double yaw, double q[4]) {
    const double cr = cos(roll * 0.5), sr = sin(roll * 0.5);
    const double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    const double cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

void ob_quat_rotate(const double q[4], const double v[3], double out[3]) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double tx = 2.0 * (y * v[2] - z * v[1]);
    const double ty = 2.0 * (z * v[0] - x * v[2]);
    const double tz = 2.0 * (x * v[1] - y * v[0]);
    out[0] = v[0] + w * tx + (y * tz - z * ty);
    out[1] = v[1] + w * ty + (z * tx - x * tz);
    out[2] = v[2] + w * tz + (x * ty - y * tx);
}

void ob_quat_between(const double from[3], const double to[3], double q[4]) {
    double a[3] = { from[0], from[1], from[2] };
    double b[3] = { to[0], to[1], to[2] };
    double na = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    double nb = sqrt(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);
    if (na < 1e-12 || nb < 1e-12) { q[0] = 1; q[1] = q[2] = q[3] = 0; return; }
    for (int i = 0; i < 3; i++) { a[i] /= na; b[i] /= nb; }

    const double dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    if (dot > 0.999999) { q[0] = 1; q[1] = q[2] = q[3] = 0; return; }
    if (dot < -0.999999) {
        // Antiparallel: any perpendicular axis gives a 180-degree turn.
        double axis[3] = { 1, 0, 0 };
        if (fabs(a[0]) > 0.9) { axis[0] = 0; axis[1] = 1; }
        double perp[3] = {
            a[1] * axis[2] - a[2] * axis[1],
            a[2] * axis[0] - a[0] * axis[2],
            a[0] * axis[1] - a[1] * axis[0],
        };
        const double n = sqrt(perp[0]*perp[0] + perp[1]*perp[1] + perp[2]*perp[2]);
        q[0] = 0.0; q[1] = perp[0] / n; q[2] = perp[1] / n; q[3] = perp[2] / n;
        return;
    }

    const double cross[3] = {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    };
    q[0] = 1.0 + dot;
    q[1] = cross[0];
    q[2] = cross[1];
    q[3] = cross[2];
    const double n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int i = 0; i < 4; i++) q[i] /= n;
}
