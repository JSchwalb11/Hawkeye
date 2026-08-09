#ifndef TEST_ORIENTATION_BASIS_H
#define TEST_ORIENTATION_BASIS_H

// An independently written MAV_SENSOR_ORIENTATION table.
//
// The `orientations` fixture is worthless if the injector asks the viewer's own
// table where the sensor points -- a wrong entry would cancel itself out and the
// test would pass. So this file transcribes the enum a second time, from the
// names, and builds rotation matrices by explicit 3x3 multiplication rather
// than through quaternions. Two separate transcriptions and two separate
// rotation implementations: a mistake in either one shows up as a mismatch.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OB_ORIENTATION_COUNT 41

// R_body<-sensor as a row-major 3x3 matrix. Returns false for out-of-range
// values, leaving `m` as the identity.
bool ob_rotation(uint8_t orientation, double m[9]);

// The sensor's +X axis expressed in body frame, i.e. the first column of the
// rotation. This is the direction a DISTANCE_SENSOR measures along.
bool ob_sensor_axis(uint8_t orientation, double axis[3]);

const char *ob_name(uint8_t orientation);

// Quaternion (w,x,y,z) for the shortest rotation taking `from` to `to`.
void ob_quat_between(const double from[3], const double to[3], double q[4]);

// Rotate a vector by a quaternion, for the injector's own use.
void ob_quat_rotate(const double q[4], const double v[3], double out[3]);

void ob_quat_mul(const double a[4], const double b[4], double out[4]);
void ob_quat_from_euler(double roll, double pitch, double yaw, double q[4]);

#ifdef __cplusplus
}
#endif

#endif
