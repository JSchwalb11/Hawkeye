#ifndef QUALITY_H
#define QUALITY_H

// Per-vehicle quality signals, all derived from messages that already exist on
// the wire. Kept free of MAVLink types so the HUD can include this header
// without dragging the dialect in.
//
// One unit trap is worth spelling out: GPS_RAW_INT.eph is HDOP x 100 and
// therefore dimensionless, while h_acc is millimetres. Drawing a ring at eph
// draws a ring in units of nothing. The accuracy ring uses h_acc; HDOP/VDOP are
// reported as numbers.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QUALITY_MAX_SATS 20

typedef struct {
    uint8_t prn;
    uint8_t used;
    uint8_t elevation_deg;
    uint8_t azimuth_deg;   // in units of 2 degrees, as MAVLink sends it
    uint8_t snr;
} sat_info_t;

typedef struct {
    // --- GPS ---
    bool     gps_valid;
    uint8_t  fix_type;             // MAV_GPS_FIX_TYPE
    uint8_t  satellites_visible;
    float    h_acc_m;              // horizontal accuracy (from h_acc, mm)
    float    v_acc_m;
    float    vel_acc_ms;
    float    hdop;                 // eph / 100
    float    vdop;                 // epv / 100
    bool     h_acc_valid;

    bool     sats_valid;
    uint8_t  sat_count;
    sat_info_t sats[QUALITY_MAX_SATS];

    // --- Position uncertainty (LOCAL_POSITION_NED_COV) ---
    bool     cov_valid;
    float    pos_cov[6];           // xx, xy, xz, yy, yz, zz in m^2

    // --- Estimator health ---
    bool     est_px4_valid;        // ESTIMATOR_STATUS: ratios, 1.0 == rejection
    float    vel_ratio, pos_horiz_ratio, pos_vert_ratio, mag_ratio, hagl_ratio, tas_ratio;
    uint16_t est_flags;

    bool     est_ardu_valid;       // EKF_STATUS_REPORT: variances
    float    ekf_velocity_variance, ekf_pos_horiz_variance, ekf_pos_vert_variance;
    float    ekf_compass_variance, ekf_terrain_alt_variance;
    uint16_t ekf_flags;

    // --- Link ---
    float    rtt_ms;               // TIMESYNC round trip
    float    rtt_ms_min;
    uint32_t seq_received;
    uint32_t seq_gaps;             // frames the sequence counter says we missed
    float    loss_pct;

    // --- Wind / vibration / terrain ---
    bool     wind_valid;
    float    wind_ned[3];
    float    wind_var_horiz, wind_var_vert;

    bool     vibe_valid;
    float    vibration[3];
    uint32_t clipping[3];

    bool     terrain_valid;
    float    terrain_height, current_height;
    uint16_t terrain_pending, terrain_loaded;

    bool     clearance_valid;
    float    bottom_clearance;     // ALTITUDE.bottom_clearance

    // --- Conflict ---
    bool     collision_reported;   // a COLLISION message arrived
    uint32_t collision_id;
    float    collision_time_to_min_delta;
    float    collision_altitude_delta;
    float    collision_horizontal_delta;
    uint8_t  collision_threat_level;
} vehicle_quality_t;

// Normalized 0..1 estimator health, where 1.0 means "at the rejection
// threshold". PX4 ratios are already normalized; ArduPilot variances are scaled
// against their own documented thresholds, so both land on one gauge.
float quality_estimator_health(const vehicle_quality_t *q);

// Worst-case one-sigma horizontal position uncertainty in metres from the
// covariance, i.e. the semi-major axis of the 1-sigma ellipse.
bool quality_horizontal_sigma(const vehicle_quality_t *q, float *semi_major_m,
                              float *semi_minor_m, float *angle_rad);

// --- Fleet separation --------------------------------------------------

typedef struct {
    int   a, b;            // vehicle indices, -1 when nothing is tracked
    float distance_m;
    float horizontal_m;
    float vertical_m;
} separation_pair_t;

// Closest pair over `count` positions given as session ENU. Positions with
// `valid[i] == false` are skipped. Returns false when fewer than two are valid.
bool quality_closest_pair(const double (*enu)[3], const bool *valid, int count,
                          separation_pair_t *out);

#ifdef __cplusplus
}
#endif

#endif
