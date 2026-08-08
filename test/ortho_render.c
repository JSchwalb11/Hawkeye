#include "ortho_render.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define COL_PANEL_BG   0x0E1116u
#define COL_PANEL_EDGE 0x2A3341u
#define COL_GRID       0x1A2130u
#define COL_TITLE      0xE6EDF3u
#define COL_SUB        0x8B98A5u
#define COL_TRUTH      0xF0F6FCu

#define COL_FREE       0x2E6F8Eu
#define COL_OCCUPIED   0xFFB454u
#define COL_OBSERVED   0x35C4A0u
#define COL_OCC_DIM    0x4A5260u
#define COL_CONTESTED  0xFF4FA3u
#define COL_FOCUS      0x4FC3FFu
#define COL_SHARED     0xF0F6FCu

const char *ortho_mode_name(ortho_mode_t m) {
    switch (m) {
        case ORTHO_OCCUPANCY:   return "OCCUPANCY";
        case ORTHO_COVERAGE:    return "COVERAGE";
        case ORTHO_DIVERGENCE:  return "DIVERGENCE";
        case ORTHO_CONTRIBUTION:return "CONTRIBUTION";
        default: return "?";
    }
}

const char *ortho_view_name(ortho_view_t v) {
    switch (v) {
        case ORTHO_TOP:   return "TOP (E-N)";
        case ORTHO_SIDE:  return "SIDE (E-U)";
        case ORTHO_FRONT: return "FRONT (N-U)";
        default: return "?";
    }
}

uint32_t ortho_vehicle_colour(int index) {
    static const uint32_t palette[8] = {
        0x4FC3FF, 0xFFD166, 0x7BE495, 0xFF6B8B,
        0xC792EA, 0x64D8CB, 0xF78C6C, 0xB0BEC5,
    };
    return palette[index & 7];
}

// ---------------------------------------------------------------- bounds

typedef struct { double min[3], max[3]; bool any; } bounds_t;

static void bounds_leaf(const om_leaf_t *leaf, void *user) {
    bounds_t *b = (bounds_t *)user;
    if (leaf->state == OM_UNKNOWN) return;
    const double h = leaf->size * 0.5;
    for (int i = 0; i < 3; i++) {
        if (!b->any || leaf->center[i] - h < b->min[i]) b->min[i] = leaf->center[i] - h;
        if (!b->any || leaf->center[i] + h > b->max[i]) b->max[i] = leaf->center[i] + h;
    }
    b->any = true;
}

void ortho_auto_bounds(const octomap_t *map, double margin, double min[3], double max[3]) {
    bounds_t b;
    memset(&b, 0, sizeof(b));
    octomap_iterate(map, bounds_leaf, &b);
    if (!b.any) {
        for (int i = 0; i < 3; i++) { min[i] = -10.0; max[i] = 10.0; }
        return;
    }
    for (int i = 0; i < 3; i++) {
        min[i] = b.min[i] - margin;
        max[i] = b.max[i] + margin;
        if (max[i] - min[i] < 1.0) { min[i] -= 0.5; max[i] += 0.5; }
    }
}

// ---------------------------------------------------------------- projection

typedef struct {
    int x0, y0, w, h;
    int ax_h, ax_v;      // world axis indices mapped to screen horizontal / vertical
    double h_min, h_span, v_min, v_span;
    double px_per_m;
} proj_t;

static void proj_setup(proj_t *p, int x0, int y0, int w, int h,
                       ortho_view_t view, const double mn[3], const double mx[3]) {
    p->x0 = x0; p->y0 = y0; p->w = w; p->h = h;
    switch (view) {
        case ORTHO_TOP:   p->ax_h = 0; p->ax_v = 1; break;   // east / north
        case ORTHO_SIDE:  p->ax_h = 0; p->ax_v = 2; break;   // east / up
        default:          p->ax_h = 1; p->ax_v = 2; break;   // north / up
    }
    const double hs = mx[p->ax_h] - mn[p->ax_h];
    const double vs = mx[p->ax_v] - mn[p->ax_v];

    // One scale for both axes so the picture is not silently stretched.
    const double s = fmin((double)w / (hs > 0 ? hs : 1.0), (double)h / (vs > 0 ? vs : 1.0));
    p->px_per_m = s;
    p->h_span = (double)w / s;
    p->v_span = (double)h / s;
    p->h_min = mn[p->ax_h] - (p->h_span - hs) * 0.5;
    p->v_min = mn[p->ax_v] - (p->v_span - vs) * 0.5;
}

static void proj_point(const proj_t *p, const double world[3], double *sx, double *sy) {
    *sx = p->x0 + (world[p->ax_h] - p->h_min) * p->px_per_m;
    // Screen y grows downward; north and up both grow upward on the page.
    *sy = p->y0 + p->h - (world[p->ax_v] - p->v_min) * p->px_per_m;
}

// ---------------------------------------------------------------- cells

typedef struct {
    canvas_t     *c;
    const proj_t *p;
    const octomap_t *map;
    ortho_mode_t  mode;
    uint32_t      focus_mask;
    int           pass;   // 0 = free/observed backdrop, 1 = occupied
} cell_ctx_t;

static void draw_cell(const om_leaf_t *leaf, void *user) {
    cell_ctx_t *cx = (cell_ctx_t *)user;
    const bool occupied = (leaf->state == OM_OCCUPIED);
    if (leaf->state == OM_UNKNOWN) return;
    if (cx->pass == 0 && occupied) return;
    if (cx->pass == 1 && !occupied) return;

    const double half = leaf->size * 0.5;
    double lo[3], hi[3];
    for (int i = 0; i < 3; i++) { lo[i] = leaf->center[i] - half; hi[i] = leaf->center[i] + half; }

    double ax, ay, bx, by;
    proj_point(cx->p, lo, &ax, &ay);
    proj_point(cx->p, hi, &bx, &by);
    int x = (int)floor(fmin(ax, bx));
    int y = (int)floor(fmin(ay, by));
    int w = (int)ceil(fabs(bx - ax));
    int hgt = (int)ceil(fabs(by - ay));
    if (w < 1) w = 1;
    if (hgt < 1) hgt = 1;
    if (x + w < cx->p->x0 || x > cx->p->x0 + cx->p->w) return;
    if (y + hgt < cx->p->y0 || y > cx->p->y0 + cx->p->h) return;

    // Clip to the panel so a cell straddling the edge cannot bleed into its
    // neighbour.
    if (x < cx->p->x0) { w -= cx->p->x0 - x; x = cx->p->x0; }
    if (y < cx->p->y0) { hgt -= cx->p->y0 - y; y = cx->p->y0; }
    if (x + w > cx->p->x0 + cx->p->w) w = cx->p->x0 + cx->p->w - x;
    if (y + hgt > cx->p->y0 + cx->p->h) hgt = cx->p->y0 + cx->p->h - y;
    if (w <= 0 || hgt <= 0) return;

    const float conf = fminf(1.0f, fabsf((float)leaf->node->log_odds) / (float)OM_LO_CLAMP);
    uint32_t colour;
    float alpha;

    switch (cx->mode) {
        case ORTHO_COVERAGE:
            // Anything that is not unknown was looked at. That is usually the
            // operational question: what did we actually see?
            colour = occupied ? COL_OCCUPIED : COL_OBSERVED;
            alpha = occupied ? 0.95f : 0.30f;
            break;

        case ORTHO_DIVERGENCE:
            if (!occupied) { colour = COL_FREE; alpha = 0.10f; break; }
            if (om_node_contested(cx->map, leaf->node)) { colour = COL_CONTESTED; alpha = 1.0f; }
            else { colour = COL_OCC_DIM; alpha = 0.75f; }
            break;

        case ORTHO_CONTRIBUTION: {
            const uint32_t obs = leaf->node->observers;
            const bool mine = (obs & cx->focus_mask) != 0;
            const bool others = (obs & ~cx->focus_mask) != 0;
            if (!occupied) { colour = COL_FREE; alpha = 0.08f; break; }
            if (mine && others) { colour = COL_SHARED; alpha = 0.95f; }
            else if (mine)      { colour = COL_FOCUS; alpha = 1.0f; }
            else                { colour = COL_OCC_DIM; alpha = 0.6f; }
            break;
        }

        default:
            colour = occupied ? COL_OCCUPIED : COL_FREE;
            alpha = occupied ? (0.35f + 0.65f * conf) : (0.10f + 0.22f * conf);
            break;
    }
    canvas_fill_rect(cx->c, x, y, w, hgt, colour, alpha);
}

// ---------------------------------------------------------------- extras

static void draw_grid(canvas_t *c, const proj_t *p) {
    // A 10 m grid, or 50 m when that would be too dense to read.
    double step = 10.0;
    while (step * p->px_per_m < 24.0) step *= 5.0;

    for (double v = ceil(p->h_min / step) * step; v < p->h_min + p->h_span; v += step) {
        const int x = (int)(p->x0 + (v - p->h_min) * p->px_per_m);
        canvas_vline(c, x, p->y0, p->y0 + p->h - 1, COL_GRID, 1.0f);
    }
    for (double v = ceil(p->v_min / step) * step; v < p->v_min + p->v_span; v += step) {
        const int y = (int)(p->y0 + p->h - (v - p->v_min) * p->px_per_m);
        canvas_hline(c, p->x0, p->x0 + p->w - 1, y, COL_GRID, 1.0f);
    }
}

// The true surface, so map and geometry can be compared by eye as well as by
// the scored numbers.
static void draw_truth(canvas_t *c, const proj_t *p, const geom_scene_t *scene, double t_s) {
    if (!scene) return;
    for (int i = 0; i < scene->count; i++) {
        const geom_plane_t *pl = &scene->planes[i];
        if (!geom_plane_active(pl, t_s)) continue;
        const int steps = 200;
        for (int a = 0; a <= steps; a++) {
            const double du = (-1.0 + 2.0 * (double)a / steps) * pl->half_u;
            for (int b = 0; b <= steps; b += steps) {
                const double dv = (-1.0 + 2.0 * (double)b / steps) * pl->half_v;
                const double w[3] = {
                    pl->point[0] + pl->u[0] * du + pl->v[0] * dv,
                    pl->point[1] + pl->u[1] * du + pl->v[1] * dv,
                    pl->point[2] + pl->u[2] * du + pl->v[2] * dv,
                };
                double sx, sy;
                proj_point(p, w, &sx, &sy);
                canvas_plot(c, (int)sx, (int)sy, COL_TRUTH, 0.55f);
            }
        }
        for (int a = 0; a <= steps; a++) {
            const double dv = (-1.0 + 2.0 * (double)a / steps) * pl->half_v;
            for (int b = 0; b <= steps; b += steps) {
                const double du = (-1.0 + 2.0 * (double)b / steps) * pl->half_u;
                const double w[3] = {
                    pl->point[0] + pl->u[0] * du + pl->v[0] * dv,
                    pl->point[1] + pl->u[1] * du + pl->v[1] * dv,
                    pl->point[2] + pl->u[2] * du + pl->v[2] * dv,
                };
                double sx, sy;
                proj_point(p, w, &sx, &sy);
                canvas_plot(c, (int)sx, (int)sy, COL_TRUTH, 0.55f);
            }
        }
    }
}

static void draw_tracks(canvas_t *c, const proj_t *p, const timeline_t *tl, int count) {
    if (!tl) return;
    for (int v = 0; v < count && v < TL_MAX_VEHICLES; v++) {
        const tl_track_t *tr = &tl->tracks[v];
        if (!tr->ring || tr->count == 0) continue;
        const uint32_t colour = ortho_vehicle_colour(v);
        const uint32_t start = (tr->head - tr->count) & (tr->cap - 1);
        for (uint32_t i = 0; i < tr->count; i++) {
            const tl_sample_t *s = &tr->ring[(start + i) & (tr->cap - 1)];
            if (!(s->flags & TL_SAMPLE_POS_VALID)) continue;
            double sx, sy;
            proj_point(p, s->enu, &sx, &sy);
            canvas_fill_rect(c, (int)sx - 1, (int)sy - 1, 2, 2, colour, 0.85f);
        }
    }
}

// ---------------------------------------------------------------- panel

void ortho_draw_panel(canvas_t *c, int x0, int y0, int w, int h,
                      const octomap_t *map, ortho_view_t view, ortho_mode_t mode,
                      const ortho_opts_t *o, const char *title) {
    const int header = 20;
    canvas_fill_rect(c, x0, y0, w, h, COL_PANEL_BG, 1.0f);
    canvas_rect_outline(c, x0, y0, w, h, COL_PANEL_EDGE);

    if (title) canvas_text(c, x0 + 6, y0 + 5, title, COL_TITLE, 1);
    canvas_text(c, x0 + w - canvas_text_width(ortho_view_name(view), 1) - 6, y0 + 5,
                ortho_view_name(view), COL_SUB, 1);

    proj_t p;
    proj_setup(&p, x0 + 1, y0 + header, w - 2, h - header - 1, view, o->min, o->max);

    draw_grid(c, &p);

    cell_ctx_t cx = { c, &p, map, mode, o->focus_mask, 0 };
    octomap_iterate(map, draw_cell, &cx);
    cx.pass = 1;
    octomap_iterate(map, draw_cell, &cx);

    draw_truth(c, &p, o->truth, o->truth_time_s);
    draw_tracks(c, &p, o->tracks, o->track_count);

    char scale[48];
    double step = 10.0;
    while (step * p.px_per_m < 24.0) step *= 5.0;
    snprintf(scale, sizeof(scale), "GRID %.0f M", step);
    canvas_text(c, x0 + 6, y0 + h - 11, scale, COL_SUB, 1);

    if (o->subtitle) {
        const int tw = canvas_text_width(o->subtitle, 1);
        canvas_text(c, x0 + w - tw - 6, y0 + h - 11, o->subtitle, COL_SUB, 1);
    }
}
