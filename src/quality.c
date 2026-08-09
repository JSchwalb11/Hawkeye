#include "quality.h"

#include <math.h>
#include <stddef.h>

// ArduPilot's EKF_STATUS_REPORT variances are "normalised" already in the sense
// that 1.0 is the failsafe threshold for velocity/position/height, while the
// compass and terrain variances use the same convention. PX4's innovation
// ratios are normalized so 1.0 == rejection. Both therefore reduce to "how
// close to 1.0 is the worst channel", which is what puts them on one gauge.
float quality_estimator_health(const vehicle_quality_t *q) {
    if (!q) return 0.0f;
    float worst = 0.0f;

    if (q->est_px4_valid) {
        const float r[6] = { q->vel_ratio, q->pos_horiz_ratio, q->pos_vert_ratio,
                             q->mag_ratio, q->hagl_ratio, q->tas_ratio };
        for (int i = 0; i < 6; i++)
            if (isfinite(r[i]) && r[i] > worst) worst = r[i];
    }
    if (q->est_ardu_valid) {
        const float v[5] = { q->ekf_velocity_variance, q->ekf_pos_horiz_variance,
                             q->ekf_pos_vert_variance, q->ekf_compass_variance,
                             q->ekf_terrain_alt_variance };
        for (int i = 0; i < 5; i++)
            if (isfinite(v[i]) && v[i] > worst) worst = v[i];
    }
    return worst;
}

bool quality_horizontal_sigma(const vehicle_quality_t *q, float *semi_major_m,
                              float *semi_minor_m, float *angle_rad) {
    if (!q || !q->cov_valid) return false;

    // Covariance is packed upper-triangular: xx, xy, xz, yy, yz, zz.
    const float xx = q->pos_cov[0];
    const float xy = q->pos_cov[1];
    const float yy = q->pos_cov[3];
    if (!isfinite(xx) || !isfinite(yy) || !isfinite(xy)) return false;
    if (xx < 0.0f || yy < 0.0f) return false;

    const float tr = xx + yy;
    const float det = xx * yy - xy * xy;
    float disc = tr * tr * 0.25f - det;
    if (disc < 0.0f) disc = 0.0f;
    const float root = sqrtf(disc);

    float l1 = tr * 0.5f + root;
    float l2 = tr * 0.5f - root;
    if (l1 < 0.0f) l1 = 0.0f;
    if (l2 < 0.0f) l2 = 0.0f;

    if (semi_major_m) *semi_major_m = sqrtf(l1);
    if (semi_minor_m) *semi_minor_m = sqrtf(l2);
    if (angle_rad) *angle_rad = 0.5f * atan2f(2.0f * xy, xx - yy);
    return true;
}

bool quality_closest_pair(const double (*enu)[3], const bool *valid, int count,
                          separation_pair_t *out) {
    if (!enu || !out || count < 2) return false;
    double best = -1.0;
    int ba = -1, bb = -1;

    for (int i = 0; i < count; i++) {
        if (valid && !valid[i]) continue;
        for (int j = i + 1; j < count; j++) {
            if (valid && !valid[j]) continue;
            const double dx = enu[i][0] - enu[j][0];
            const double dy = enu[i][1] - enu[j][1];
            const double dz = enu[i][2] - enu[j][2];
            const double d2 = dx * dx + dy * dy + dz * dz;
            if (best < 0.0 || d2 < best) { best = d2; ba = i; bb = j; }
        }
    }
    if (ba < 0) return false;

    const double dx = enu[ba][0] - enu[bb][0];
    const double dy = enu[ba][1] - enu[bb][1];
    const double dz = enu[ba][2] - enu[bb][2];
    out->a = ba;
    out->b = bb;
    out->distance_m = (float)sqrt(best);
    out->horizontal_m = (float)sqrt(dx * dx + dy * dy);
    out->vertical_m = (float)fabs(dz);
    return true;
}
