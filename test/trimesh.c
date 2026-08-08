#include "trimesh.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void clampi(int *v, int lo, int hi) {
    if (*v < lo) *v = lo;
    if (*v > hi) *v = hi;
}

static int grid_index(const trimesh_t *m, int ix, int iy, int iz) {
    return (iz * m->dim[1] + iy) * m->dim[0] + ix;
}

// Squared distance from a point to a triangle. Ericson's region test: find the
// barycentric coordinates of the closest point, clamped to the triangle.
static double point_tri_dist2(const double p[3], const float t[9]) {
    const double a[3] = { t[0], t[1], t[2] };
    const double b[3] = { t[3], t[4], t[5] };
    const double c[3] = { t[6], t[7], t[8] };

    const double ab[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
    const double ac[3] = { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
    const double ap[3] = { p[0] - a[0], p[1] - a[1], p[2] - a[2] };

    const double d1 = ab[0]*ap[0] + ab[1]*ap[1] + ab[2]*ap[2];
    const double d2 = ac[0]*ap[0] + ac[1]*ap[1] + ac[2]*ap[2];

    double q[3];
    if (d1 <= 0.0 && d2 <= 0.0) {
        q[0] = a[0]; q[1] = a[1]; q[2] = a[2];
    } else {
        const double bp[3] = { p[0] - b[0], p[1] - b[1], p[2] - b[2] };
        const double d3 = ab[0]*bp[0] + ab[1]*bp[1] + ab[2]*bp[2];
        const double d4 = ac[0]*bp[0] + ac[1]*bp[1] + ac[2]*bp[2];
        if (d3 >= 0.0 && d4 <= d3) {
            q[0] = b[0]; q[1] = b[1]; q[2] = b[2];
        } else {
            const double vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
                const double v = d1 / (d1 - d3);
                for (int k = 0; k < 3; k++) q[k] = a[k] + v * ab[k];
            } else {
                const double cp[3] = { p[0] - c[0], p[1] - c[1], p[2] - c[2] };
                const double d5 = ab[0]*cp[0] + ab[1]*cp[1] + ab[2]*cp[2];
                const double d6 = ac[0]*cp[0] + ac[1]*cp[1] + ac[2]*cp[2];
                if (d6 >= 0.0 && d5 <= d6) {
                    q[0] = c[0]; q[1] = c[1]; q[2] = c[2];
                } else {
                    const double vb = d5 * d2 - d1 * d6;
                    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
                        const double w = d2 / (d2 - d6);
                        for (int k = 0; k < 3; k++) q[k] = a[k] + w * ac[k];
                    } else {
                        const double va = d3 * d6 - d5 * d4;
                        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
                            const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                            for (int k = 0; k < 3; k++) q[k] = b[k] + w * (c[k] - b[k]);
                        } else {
                            const double denom = 1.0 / (va + vb + vc);
                            const double v = vb * denom, w = vc * denom;
                            for (int k = 0; k < 3; k++) q[k] = a[k] + ab[k] * v + ac[k] * w;
                        }
                    }
                }
            }
        }
    }

    const double dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2];
    return dx*dx + dy*dy + dz*dz;
}

static int build_grid(trimesh_t *m) {
    // Aim for a cell a little larger than the mean triangle, so most triangles
    // land in a handful of cells and a query touches few.
    double edge_sum = 0.0;
    for (uint32_t i = 0; i < m->count; i++) {
        const float *t = m->tri[i];
        double lo[3] = { t[0], t[1], t[2] }, hi[3] = { t[0], t[1], t[2] };
        for (int v = 1; v < 3; v++)
            for (int k = 0; k < 3; k++) {
                const double c = t[v * 3 + k];
                if (c < lo[k]) lo[k] = c;
                if (c > hi[k]) hi[k] = c;
            }
        double big = hi[0] - lo[0];
        if (hi[1] - lo[1] > big) big = hi[1] - lo[1];
        if (hi[2] - lo[2] > big) big = hi[2] - lo[2];
        edge_sum += big;
    }
    m->cell = m->count ? (edge_sum / (double)m->count) : 1.0;
    if (m->cell < 1e-3) m->cell = 1e-3;

    for (int k = 0; k < 3; k++) {
        const double span = m->hi[k] - m->lo[k];
        m->dim[k] = (int)(span / m->cell) + 1;
        if (m->dim[k] < 1) m->dim[k] = 1;
        if (m->dim[k] > 256) {
            m->cell = span / 256.0;
            m->dim[k] = 256;
        }
    }
    // One cell size for all axes, so recompute after any clamp above.
    for (int k = 0; k < 3; k++) {
        const double span = m->hi[k] - m->lo[k];
        m->dim[k] = (int)(span / m->cell) + 1;
        if (m->dim[k] < 1) m->dim[k] = 1;
    }

    const size_t cells = (size_t)m->dim[0] * m->dim[1] * m->dim[2];
    m->bucket_start = (uint32_t *)calloc(cells + 1, sizeof(uint32_t));
    if (!m->bucket_start) return -1;

    // Two passes: count, then fill. A triangle goes in every cell its bounding
    // box touches, so no query can miss it.
    size_t total = 0;
    for (int pass = 0; pass < 2; pass++) {
        if (pass == 1) {
            for (size_t i = 0; i < cells; i++) m->bucket_start[i + 1] += m->bucket_start[i];
            total = m->bucket_start[cells];
            m->bucket_item = (uint32_t *)malloc(total ? total * sizeof(uint32_t) : 1);
            if (!m->bucket_item) return -1;
        }
        uint32_t *cursor = NULL;
        if (pass == 1) {
            cursor = (uint32_t *)malloc(cells * sizeof(uint32_t));
            if (!cursor) return -1;
            memcpy(cursor, m->bucket_start, cells * sizeof(uint32_t));
        }
        for (uint32_t i = 0; i < m->count; i++) {
            const float *t = m->tri[i];
            int lo_i[3], hi_i[3];
            for (int k = 0; k < 3; k++) {
                double lo = t[k], hi = t[k];
                for (int v = 1; v < 3; v++) {
                    const double c = t[v * 3 + k];
                    if (c < lo) lo = c;
                    if (c > hi) hi = c;
                }
                lo_i[k] = (int)((lo - m->lo[k]) / m->cell);
                hi_i[k] = (int)((hi - m->lo[k]) / m->cell);
                clampi(&lo_i[k], 0, m->dim[k] - 1);
                clampi(&hi_i[k], 0, m->dim[k] - 1);
            }
            for (int iz = lo_i[2]; iz <= hi_i[2]; iz++)
            for (int iy = lo_i[1]; iy <= hi_i[1]; iy++)
            for (int ix = lo_i[0]; ix <= hi_i[0]; ix++) {
                const int g = grid_index(m, ix, iy, iz);
                if (pass == 0) m->bucket_start[g + 1]++;
                else m->bucket_item[cursor[g]++] = i;
            }
        }
        free(cursor);
    }
    return 0;
}

int trimesh_load(trimesh_t *m, const char *path, char *err, size_t err_len) {
    if (!m || !path) return -1;
    memset(m, 0, sizeof(*m));

    FILE *f = fopen(path, "rb");
    if (!f) { if (err) snprintf(err, err_len, "cannot open %s", path); return -1; }

    char magic[8];
    uint32_t count = 0, reserved = 0;
    if (fread(magic, 1, 8, f) != 8 || memcmp(magic, "HKTRI1\0\0", 8) != 0) {
        fclose(f);
        if (err) snprintf(err, err_len, "%s is not a HKTRI1 mesh", path);
        return -1;
    }
    if (fread(&count, 4, 1, f) != 1 || fread(&reserved, 4, 1, f) != 1 || count == 0) {
        fclose(f);
        if (err) snprintf(err, err_len, "%s has no triangles", path);
        return -1;
    }

    m->tri = (float (*)[9])malloc((size_t)count * 9 * sizeof(float));
    if (!m->tri) { fclose(f); return -1; }
    if (fread(m->tri, 9 * sizeof(float), count, f) != count) {
        free(m->tri); m->tri = NULL; fclose(f);
        if (err) snprintf(err, err_len, "short read on %s", path);
        return -1;
    }
    fclose(f);
    m->count = count;

    for (uint32_t i = 0; i < count; i++)
        for (int v = 0; v < 3; v++)
            for (int k = 0; k < 3; k++) {
                const double c = m->tri[i][v * 3 + k];
                if (i == 0 && v == 0) { m->lo[k] = c; m->hi[k] = c; }
                else { if (c < m->lo[k]) m->lo[k] = c; if (c > m->hi[k]) m->hi[k] = c; }
            }

    if (build_grid(m) != 0) {
        if (err) snprintf(err, err_len, "out of memory building the mesh grid");
        trimesh_free(m);
        return -1;
    }
    return 0;
}

void trimesh_free(trimesh_t *m) {
    if (!m) return;
    free(m->tri); free(m->bucket_start); free(m->bucket_item);
    memset(m, 0, sizeof(*m));
}

double trimesh_distance(const trimesh_t *m, const double p[3]) {
    if (!m || !m->tri || !m->bucket_start) return 1e18;

    int base[3];
    for (int k = 0; k < 3; k++) {
        base[k] = (int)((p[k] - m->lo[k]) / m->cell);
        clampi(&base[k], 0, m->dim[k] - 1);
    }

    double best2 = 1e36;
    // Expanding shells. A shell at radius r cannot hold anything closer than
    // (r-1) cells, so once the best is inside that the search is done.
    for (int r = 0; r < 512; r++) {
        if (best2 < 1e35) {
            const double floor_d = (double)(r - 1) * m->cell;
            if (floor_d > 0.0 && best2 <= floor_d * floor_d) break;
        }
        bool any = false;
        for (int dz = -r; dz <= r; dz++)
        for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++) {
            if (r > 0 && abs(dx) != r && abs(dy) != r && abs(dz) != r) continue;
            const int ix = base[0] + dx, iy = base[1] + dy, iz = base[2] + dz;
            if (ix < 0 || iy < 0 || iz < 0 ||
                ix >= m->dim[0] || iy >= m->dim[1] || iz >= m->dim[2]) continue;
            any = true;
            const int g = grid_index(m, ix, iy, iz);
            for (uint32_t k = m->bucket_start[g]; k < m->bucket_start[g + 1]; k++) {
                const double d2 = point_tri_dist2(p, m->tri[m->bucket_item[k]]);
                if (d2 < best2) best2 = d2;
            }
        }
        // Once the shells have left the grid entirely there is nothing more.
        if (!any && r > 0 && best2 < 1e35) break;
        if (!any && r > m->dim[0] + m->dim[1] + m->dim[2]) break;
    }
    return (best2 < 1e35) ? sqrt(best2) : 1e18;
}

// ------------------------------------------------------------ ray casting

static bool ray_tri(const double o[3], const double dir[3], const float t[9],
                    double *out_t) {
    const double e1[3] = { t[3] - t[0], t[4] - t[1], t[5] - t[2] };
    const double e2[3] = { t[6] - t[0], t[7] - t[1], t[8] - t[2] };
    const double pv[3] = { dir[1]*e2[2] - dir[2]*e2[1],
                           dir[2]*e2[0] - dir[0]*e2[2],
                           dir[0]*e2[1] - dir[1]*e2[0] };
    const double det = e1[0]*pv[0] + e1[1]*pv[1] + e1[2]*pv[2];
    if (fabs(det) < 1e-12) return false;          // parallel
    const double inv = 1.0 / det;
    const double tv[3] = { o[0] - t[0], o[1] - t[1], o[2] - t[2] };
    const double u = (tv[0]*pv[0] + tv[1]*pv[1] + tv[2]*pv[2]) * inv;
    if (u < 0.0 || u > 1.0) return false;
    const double qv[3] = { tv[1]*e1[2] - tv[2]*e1[1],
                           tv[2]*e1[0] - tv[0]*e1[2],
                           tv[0]*e1[1] - tv[1]*e1[0] };
    const double v = (dir[0]*qv[0] + dir[1]*qv[1] + dir[2]*qv[2]) * inv;
    if (v < 0.0 || u + v > 1.0) return false;
    const double tt = (e2[0]*qv[0] + e2[1]*qv[1] + e2[2]*qv[2]) * inv;
    if (tt <= 0.0) return false;
    *out_t = tt;
    return true;
}

bool trimesh_raycast(const trimesh_t *m, const double o[3], const double d[3],
                     double max_dist, double *out_t) {
    if (!m || !m->tri || !m->bucket_start) return false;

    // Clip to the mesh box so the walk starts where there is something to find.
    double t0 = 0.0, t1 = max_dist;
    for (int k = 0; k < 3; k++) {
        if (fabs(d[k]) < 1e-12) {
            if (o[k] < m->lo[k] || o[k] > m->hi[k]) return false;
            continue;
        }
        double a = (m->lo[k] - o[k]) / d[k];
        double b = (m->hi[k] - o[k]) / d[k];
        if (a > b) { const double s = a; a = b; b = s; }
        if (a > t0) t0 = a;
        if (b < t1) t1 = b;
        if (t0 > t1) return false;
    }

    int idx[3], step[3];
    double tmax[3], tdelta[3];
    double p[3];
    for (int k = 0; k < 3; k++) p[k] = o[k] + d[k] * (t0 + 1e-9);
    for (int k = 0; k < 3; k++) {
        idx[k] = (int)((p[k] - m->lo[k]) / m->cell);
        clampi(&idx[k], 0, m->dim[k] - 1);
        if (d[k] > 1e-12) {
            step[k] = 1;
            tmax[k] = (m->lo[k] + (idx[k] + 1) * m->cell - o[k]) / d[k];
            tdelta[k] = m->cell / d[k];
        } else if (d[k] < -1e-12) {
            step[k] = -1;
            tmax[k] = (m->lo[k] + idx[k] * m->cell - o[k]) / d[k];
            tdelta[k] = -m->cell / d[k];
        } else {
            step[k] = 0; tmax[k] = 1e300; tdelta[k] = 1e300;
        }
    }

    double best = 1e300;
    double t = t0;
    while (t <= t1) {
        const int g = grid_index(m, idx[0], idx[1], idx[2]);
        for (uint32_t k = m->bucket_start[g]; k < m->bucket_start[g + 1]; k++) {
            double tt;
            if (!ray_tri(o, d, m->tri[m->bucket_item[k]], &tt)) continue;
            if (tt < best && tt <= max_dist) best = tt;
        }
        // A hit inside the cell just visited cannot be beaten by a later cell,
        // so stop once the nearest hit is closer than the next boundary.
        int axis = 0;
        if (tmax[1] < tmax[axis]) axis = 1;
        if (tmax[2] < tmax[axis]) axis = 2;
        if (best <= tmax[axis]) break;
        if (step[axis] == 0) break;
        idx[axis] += step[axis];
        if (idx[axis] < 0 || idx[axis] >= m->dim[axis]) break;
        t = tmax[axis];
        tmax[axis] += tdelta[axis];
    }

    if (best > max_dist || best >= 1e299) return false;
    if (out_t) *out_t = best;
    return true;
}
