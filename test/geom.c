#include "geom.h"

#include <math.h>
#include <string.h>

static void v_normalize(double v[3]) {
    const double n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n > 1e-12) { v[0] /= n; v[1] /= n; v[2] /= n; }
}

static double v_dot(const double a[3], const double b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void v_cross(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

void geom_plane_make(geom_plane_t *p, const double point[3], const double normal[3],
                     double half_u, double half_v, const char *label) {
    memset(p, 0, sizeof(*p));
    memcpy(p->point, point, sizeof(p->point));
    memcpy(p->normal, normal, sizeof(p->normal));
    v_normalize(p->normal);

    // Any axis not parallel to the normal seeds the in-plane basis.
    double seed[3] = { 0.0, 0.0, 1.0 };
    if (fabs(v_dot(p->normal, seed)) > 0.9) { seed[0] = 1.0; seed[1] = 0.0; seed[2] = 0.0; }
    v_cross(p->normal, seed, p->u);
    v_normalize(p->u);
    v_cross(p->normal, p->u, p->v);
    v_normalize(p->v);

    p->half_u = half_u;
    p->half_v = half_v;
    p->active_from_s = 0.0;
    p->active_to_s = 1e18;
    if (label) { strncpy(p->label, label, sizeof(p->label) - 1); }
}

void geom_plane_set_lifetime(geom_plane_t *p, double from_s, double to_s) {
    p->active_from_s = from_s;
    p->active_to_s = to_s;
}

static bool plane_raycast(const geom_plane_t *p, const double o[3], const double d[3],
                          double max_t, double *t_hit) {
    const double denom = v_dot(p->normal, d);
    if (fabs(denom) < 1e-9) return false;

    double rel[3] = { p->point[0] - o[0], p->point[1] - o[1], p->point[2] - o[2] };
    const double t = v_dot(p->normal, rel) / denom;
    if (t < 1e-6 || t > max_t) return false;

    const double hit[3] = { o[0] + d[0] * t, o[1] + d[1] * t, o[2] + d[2] * t };
    const double loc[3] = { hit[0] - p->point[0], hit[1] - p->point[1], hit[2] - p->point[2] };
    if (fabs(v_dot(loc, p->u)) > p->half_u) return false;
    if (fabs(v_dot(loc, p->v)) > p->half_v) return false;

    *t_hit = t;
    return true;
}

bool geom_scene_raycast(const geom_scene_t *s, double t_s, const double origin[3],
                        const double dir[3], double max_t, double *t_hit, int *plane_idx) {
    double best = max_t;
    int best_i = -1;
    for (int i = 0; i < s->count; i++) {
        if (!geom_plane_active(&s->planes[i], t_s)) continue;
        double t;
        if (plane_raycast(&s->planes[i], origin, dir, best, &t) && t < best) {
            best = t;
            best_i = i;
        }
    }
    if (best_i < 0) return false;
    if (t_hit) *t_hit = best;
    if (plane_idx) *plane_idx = best_i;
    return true;
}

static double plane_distance(const geom_plane_t *p, const double q[3]) {
    const double loc[3] = { q[0] - p->point[0], q[1] - p->point[1], q[2] - p->point[2] };
    double du = v_dot(loc, p->u);
    double dv = v_dot(loc, p->v);
    const double dn = v_dot(loc, p->normal);

    // Clamp into the patch, then measure to that clamped point.
    if (du >  p->half_u) du =  p->half_u;
    if (du < -p->half_u) du = -p->half_u;
    if (dv >  p->half_v) dv =  p->half_v;
    if (dv < -p->half_v) dv = -p->half_v;

    const double nearest[3] = {
        p->point[0] + p->u[0] * du + p->v[0] * dv,
        p->point[1] + p->u[1] * du + p->v[1] * dv,
        p->point[2] + p->u[2] * du + p->v[2] * dv,
    };
    const double dx = q[0] - nearest[0], dy = q[1] - nearest[1], dz = q[2] - nearest[2];
    (void)dn;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

double geom_scene_distance(const geom_scene_t *s, double t_s, const double p[3]) {
    double best = 1e18;
    for (int i = 0; i < s->count; i++) {
        if (!geom_plane_active(&s->planes[i], t_s)) continue;
        const double d = plane_distance(&s->planes[i], p);
        if (d < best) best = d;
    }
    return best;
}

bool geom_scene_on_surface(const geom_scene_t *s, double t_s, const double p[3], double tol) {
    return geom_scene_distance(s, t_s, p) <= tol;
}
