#include "splat.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The on-disk record: position f32x3, scale f32x3, colour u8x4, rotation
// quaternion u8x4 encoded as round(q * 128) + 128. Thirty-two bytes, no
// header -- the layout every splat viewer reads.
#define SPLAT_RECORD_BYTES 32

static void quat_to_mat(const float q[4], float m[9]) {
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    m[0] = 1 - 2 * (y * y + z * z); m[1] = 2 * (x * y - w * z);     m[2] = 2 * (x * z + w * y);
    m[3] = 2 * (x * y + w * z);     m[4] = 1 - 2 * (x * x + z * z); m[5] = 2 * (y * z - w * x);
    m[6] = 2 * (x * z - w * y);     m[7] = 2 * (y * z + w * x);     m[8] = 1 - 2 * (x * x + y * y);
}

// Sigma^-1 = R diag(1/s^2) R^T, kept as the six distinct entries.
static void build_inv_cov(const float m[9], const float s[3], float out[6]) {
    const float is[3] = { 1.0f / (s[0] * s[0]), 1.0f / (s[1] * s[1]), 1.0f / (s[2] * s[2]) };
    float a[9];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            float v = 0.0f;
            for (int k = 0; k < 3; k++) v += m[r * 3 + k] * is[k] * m[c * 3 + k];
            a[r * 3 + c] = v;
        }
    out[0] = a[0]; out[1] = a[1]; out[2] = a[2];
    out[3] = a[4]; out[4] = a[5]; out[5] = a[8];
}

static double quad(const float A[6], const double u[3], const double v[3]) {
    return u[0] * (A[0] * v[0] + A[1] * v[1] + A[2] * v[2])
         + u[1] * (A[1] * v[0] + A[3] * v[1] + A[4] * v[2])
         + u[2] * (A[2] * v[0] + A[4] * v[1] + A[5] * v[2]);
}

static int grid_index(const splat_cloud_t *c, int ix, int iy, int iz) {
    return (iz * c->dim[1] + iy) * c->dim[0] + ix;
}

static void clampi(int *v, int lo, int hi) {
    if (*v < lo) *v = lo;
    if (*v > hi) *v = hi;
}

static int build_grid(splat_cloud_t *c) {
    // One cell per few splat radii: small enough that a ray only visits cells
    // it could plausibly intersect, large enough that the table stays cheap.
    double rsum = 0.0;
    for (uint32_t i = 0; i < c->count; i++) rsum += c->s[i].radius;
    c->cell = (c->count ? rsum / (double)c->count : 1.0) * 2.0;
    if (c->cell < 1e-3) c->cell = 1e-3;

    for (int k = 0; k < 3; k++) {
        const double span = c->hi[k] - c->lo[k];
        c->dim[k] = (int)(span / c->cell) + 1;
        if (c->dim[k] < 1) c->dim[k] = 1;
        if (c->dim[k] > 512) c->dim[k] = 512;
    }
    // Recompute the effective cell so the grid actually spans the box after the
    // clamp above; otherwise a tall cloud would index off the end.
    for (int k = 0; k < 3; k++) {
        const double span = c->hi[k] - c->lo[k];
        const double need = span / (double)c->dim[k];
        if (need > c->cell) c->cell = need;
    }
    for (int k = 0; k < 3; k++) {
        const double span = c->hi[k] - c->lo[k];
        c->dim[k] = (int)(span / c->cell) + 1;
        if (c->dim[k] < 1) c->dim[k] = 1;
    }

    const size_t cells = (size_t)c->dim[0] * c->dim[1] * c->dim[2];
    c->bucket_start = (uint32_t *)calloc(cells + 1, sizeof(uint32_t));
    c->bucket_item = (uint32_t *)malloc((size_t)c->count * sizeof(uint32_t));
    if (!c->bucket_start || !c->bucket_item) return -1;

    for (uint32_t i = 0; i < c->count; i++) {
        int ix = (int)((c->s[i].pos[0] - c->lo[0]) / c->cell);
        int iy = (int)((c->s[i].pos[1] - c->lo[1]) / c->cell);
        int iz = (int)((c->s[i].pos[2] - c->lo[2]) / c->cell);
        clampi(&ix, 0, c->dim[0] - 1); clampi(&iy, 0, c->dim[1] - 1); clampi(&iz, 0, c->dim[2] - 1);
        c->bucket_start[grid_index(c, ix, iy, iz) + 1]++;
    }
    for (size_t i = 0; i < cells; i++) c->bucket_start[i + 1] += c->bucket_start[i];

    uint32_t *cursor = (uint32_t *)malloc(cells * sizeof(uint32_t));
    if (!cursor) return -1;
    memcpy(cursor, c->bucket_start, cells * sizeof(uint32_t));
    for (uint32_t i = 0; i < c->count; i++) {
        int ix = (int)((c->s[i].pos[0] - c->lo[0]) / c->cell);
        int iy = (int)((c->s[i].pos[1] - c->lo[1]) / c->cell);
        int iz = (int)((c->s[i].pos[2] - c->lo[2]) / c->cell);
        clampi(&ix, 0, c->dim[0] - 1); clampi(&iy, 0, c->dim[1] - 1); clampi(&iz, 0, c->dim[2] - 1);
        c->bucket_item[cursor[grid_index(c, ix, iy, iz)]++] = i;
    }
    free(cursor);
    return 0;
}

int splat_load(splat_cloud_t *c, const char *path, char *err, size_t err_len) {
    if (!c || !path) return -1;
    memset(c, 0, sizeof(*c));

    FILE *f = fopen(path, "rb");
    if (!f) { if (err) snprintf(err, err_len, "cannot open %s", path); return -1; }
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return -1; }
    const long size = ftell(f);
    rewind(f);
    if (size <= 0 || (size % SPLAT_RECORD_BYTES) != 0) {
        fclose(f);
        if (err) snprintf(err, err_len, "%s is %ld bytes, not a multiple of %d",
                          path, size, SPLAT_RECORD_BYTES);
        return -1;
    }

    const uint32_t n = (uint32_t)(size / SPLAT_RECORD_BYTES);
    uint8_t *raw = (uint8_t *)malloc((size_t)size);
    c->s = (splat_t *)calloc(n, sizeof(splat_t));
    if (!raw || !c->s) { free(raw); fclose(f); return -1; }
    if (fread(raw, 1, (size_t)size, f) != (size_t)size) {
        free(raw); fclose(f);
        if (err) snprintf(err, err_len, "short read on %s", path);
        return -1;
    }
    fclose(f);

    c->count = n;
    for (uint32_t i = 0; i < n; i++) {
        const uint8_t *p = raw + (size_t)i * SPLAT_RECORD_BYTES;
        float pos[3], scale[3], q[4];
        memcpy(pos, p, 12);
        memcpy(scale, p + 12, 12);
        const uint8_t *col = p + 24;
        const uint8_t *rot = p + 28;
        for (int k = 0; k < 4; k++) q[k] = ((float)rot[k] - 128.0f) / 128.0f;
        const float qn = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (qn > 1e-6f) for (int k = 0; k < 4; k++) q[k] /= qn;
        else { q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f; }

        for (int k = 0; k < 3; k++) if (!(scale[k] > 1e-6f)) scale[k] = 1e-6f;

        float m[9];
        quat_to_mat(q, m);
        splat_t *s = &c->s[i];
        memcpy(s->pos, pos, sizeof(pos));
        build_inv_cov(m, scale, s->inv_cov);
        s->alpha = (float)col[3] / 255.0f;
        memcpy(s->col, col, 4);
        const float smax = fmaxf(scale[0], fmaxf(scale[1], scale[2]));
        s->radius = 3.0f * smax;

        for (int k = 0; k < 3; k++) {
            const double a = pos[k] - s->radius, b = pos[k] + s->radius;
            if (i == 0) { c->lo[k] = a; c->hi[k] = b; }
            else { if (a < c->lo[k]) c->lo[k] = a; if (b > c->hi[k]) c->hi[k] = b; }
        }
    }
    free(raw);

    if (build_grid(c) != 0) {
        if (err) snprintf(err, err_len, "out of memory building the splat grid");
        splat_free(c);
        return -1;
    }
    return 0;
}

void splat_free(splat_cloud_t *c) {
    if (!c) return;
    free(c->s); free(c->bucket_start); free(c->bucket_item);
    memset(c, 0, sizeof(*c));
}

// ------------------------------------------------------------- ray casting

typedef struct { double t; float alpha; } hit_t;

static int cmp_hit(const void *a, const void *b) {
    const double ta = ((const hit_t *)a)->t, tb = ((const hit_t *)b)->t;
    return (ta < tb) ? -1 : (ta > tb) ? 1 : 0;
}

// Clip the ray to the cloud's bounding box so the traversal starts where there
// is something to find.
static bool clip_to_box(const splat_cloud_t *c, const double o[3], const double d[3],
                        double *t0, double *t1) {
    double lo = 0.0, hi = *t1;
    for (int k = 0; k < 3; k++) {
        if (fabs(d[k]) < 1e-12) {
            if (o[k] < c->lo[k] || o[k] > c->hi[k]) return false;
            continue;
        }
        double a = (c->lo[k] - o[k]) / d[k];
        double b = (c->hi[k] - o[k]) / d[k];
        if (a > b) { const double s = a; a = b; b = s; }
        if (a > lo) lo = a;
        if (b < hi) hi = b;
        if (lo > hi) return false;
    }
    *t0 = lo; *t1 = hi;
    return true;
}

#define SPLAT_MAX_HITS 8192

bool splat_raycast(const splat_cloud_t *c, const double o[3], const double d[3],
                   double max_dist, double *out_t) {
    if (!c || !c->s || !c->bucket_start) return false;

    double t0 = 0.0, t1 = max_dist;
    if (!clip_to_box(c, o, d, &t0, &t1)) return false;

    static hit_t hits[SPLAT_MAX_HITS];
    int nhit = 0;

    // Walk the grid with a 3D DDA and evaluate the splats in each cell. Cells
    // are visited in depth order, but splats within one are not, so the hits
    // are sorted before compositing rather than assumed ordered.
    double p[3];
    for (int k = 0; k < 3; k++) p[k] = o[k] + d[k] * (t0 + 1e-6);

    int idx[3], step[3];
    double tmax[3], tdelta[3];
    for (int k = 0; k < 3; k++) {
        idx[k] = (int)((p[k] - c->lo[k]) / c->cell);
        clampi(&idx[k], 0, c->dim[k] - 1);
        if (d[k] > 1e-12) {
            step[k] = 1;
            const double next = c->lo[k] + (idx[k] + 1) * c->cell;
            tmax[k] = (next - o[k]) / d[k];
            tdelta[k] = c->cell / d[k];
        } else if (d[k] < -1e-12) {
            step[k] = -1;
            const double next = c->lo[k] + idx[k] * c->cell;
            tmax[k] = (next - o[k]) / d[k];
            tdelta[k] = -c->cell / d[k];
        } else {
            step[k] = 0;
            tmax[k] = 1e300;
            tdelta[k] = 1e300;
        }
    }

    double t = t0;
    while (t <= t1 && nhit < SPLAT_MAX_HITS) {
        const uint32_t g = (uint32_t)grid_index(c, idx[0], idx[1], idx[2]);
        for (uint32_t k = c->bucket_start[g]; k < c->bucket_start[g + 1]; k++) {
            const splat_t *s = &c->s[c->bucket_item[k]];
            const double e[3] = { o[0] - s->pos[0], o[1] - s->pos[1], o[2] - s->pos[2] };
            const double dAd = quad(s->inv_cov, d, d);
            if (dAd <= 1e-12) continue;
            const double dAe = quad(s->inv_cov, d, e);
            const double tstar = -dAe / dAd;
            if (tstar < t0 || tstar > t1) continue;
            // Peak of the Gaussian along the ray, in closed form: the residual
            // exponent once the along-ray component is minimised out.
            const double eAe = quad(s->inv_cov, e, e);
            const double resid = eAe - (dAe * dAe) / dAd;
            if (resid > 18.0) continue;   // beyond 3 sigma off-axis: no contribution
            const double a = (double)s->alpha * exp(-0.5 * resid);
            if (a < 0.004) continue;
            hits[nhit].t = tstar;
            hits[nhit].alpha = (float)(a > 0.999 ? 0.999 : a);
            nhit++;
            if (nhit >= SPLAT_MAX_HITS) break;
        }

        // Step to the next cell.
        int axis = 0;
        if (tmax[1] < tmax[axis]) axis = 1;
        if (tmax[2] < tmax[axis]) axis = 2;
        if (step[axis] == 0) break;
        idx[axis] += step[axis];
        if (idx[axis] < 0 || idx[axis] >= c->dim[axis]) break;
        t = tmax[axis];
        tmax[axis] += tdelta[axis];
    }

    if (nhit == 0) return false;
    qsort(hits, (size_t)nhit, sizeof(hit_t), cmp_hit);

    double transmittance = 1.0;
    for (int i = 0; i < nhit; i++) {
        transmittance *= (1.0 - (double)hits[i].alpha);
        if (1.0 - transmittance >= SPLAT_SURFACE_ALPHA) {
            if (out_t) *out_t = hits[i].t;
            return true;
        }
    }
    return false;
}

// ------------------------------------------------------------- proximity

double splat_distance(const splat_cloud_t *c, const double p[3]) {
    if (!c || !c->s || !c->bucket_start) return 1e18;

    int base[3];
    for (int k = 0; k < 3; k++) {
        base[k] = (int)((p[k] - c->lo[k]) / c->cell);
        clampi(&base[k], 0, c->dim[k] - 1);
    }

    double best = 1e18;
    // Expanding shells: stop as soon as the nearest possible splat in the next
    // shell could not beat what has already been found.
    for (int r = 0; r < 64; r++) {
        if (best < 1e17 && best <= (double)(r - 1) * c->cell) break;
        bool any_cell = false;
        for (int dz = -r; dz <= r; dz++)
        for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++) {
            if (r > 0 && abs(dx) != r && abs(dy) != r && abs(dz) != r) continue;
            const int ix = base[0] + dx, iy = base[1] + dy, iz = base[2] + dz;
            if (ix < 0 || iy < 0 || iz < 0 ||
                ix >= c->dim[0] || iy >= c->dim[1] || iz >= c->dim[2]) continue;
            any_cell = true;
            const uint32_t g = (uint32_t)grid_index(c, ix, iy, iz);
            for (uint32_t k = c->bucket_start[g]; k < c->bucket_start[g + 1]; k++) {
                const splat_t *s = &c->s[c->bucket_item[k]];
                const double ddx = p[0] - s->pos[0];
                const double ddy = p[1] - s->pos[1];
                const double ddz = p[2] - s->pos[2];
                const double dd = ddx * ddx + ddy * ddy + ddz * ddz;
                if (dd < best * best) best = sqrt(dd);
            }
        }
        if (!any_cell && r > 0 && best < 1e17) break;
    }
    return best;
}

bool splat_cell_occupied(const splat_cloud_t *c, const double p[3], double size) {
    if (!c || !c->s || !c->bucket_start) return false;
    const double h = size * 0.5;
    int lo_i[3], hi_i[3];
    for (int k = 0; k < 3; k++) {
        lo_i[k] = (int)((p[k] - h - c->lo[k]) / c->cell);
        hi_i[k] = (int)((p[k] + h - c->lo[k]) / c->cell);
        clampi(&lo_i[k], 0, c->dim[k] - 1);
        clampi(&hi_i[k], 0, c->dim[k] - 1);
    }
    for (int iz = lo_i[2]; iz <= hi_i[2]; iz++)
    for (int iy = lo_i[1]; iy <= hi_i[1]; iy++)
    for (int ix = lo_i[0]; ix <= hi_i[0]; ix++) {
        const uint32_t g = (uint32_t)grid_index(c, ix, iy, iz);
        for (uint32_t k = c->bucket_start[g]; k < c->bucket_start[g + 1]; k++) {
            const splat_t *s = &c->s[c->bucket_item[k]];
            if (fabs(s->pos[0] - p[0]) <= h &&
                fabs(s->pos[1] - p[1]) <= h &&
                fabs(s->pos[2] - p[2]) <= h) return true;
        }
    }
    return false;
}
