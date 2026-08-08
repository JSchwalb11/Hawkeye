#include "truth.h"

#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------- write

int truth_writer_open(truth_writer_t *w, const char *prefix) {
    if (!w || !prefix) return -1;
    memset(w, 0, sizeof(*w));
    snprintf(w->prefix, sizeof(w->prefix), "%s", prefix);

    char path[600];
    snprintf(path, sizeof(path), "%s.rays", prefix);
    w->rays = fopen(path, "wb");
    if (!w->rays) return -1;

    // Header is rewritten at finish, once the count is known.
    const uint32_t hdr[2] = { TRUTH_RAYS_MAGIC, 0 };
    if (fwrite(hdr, sizeof(hdr), 1, w->rays) != 1) return -1;
    return 0;
}

int truth_writer_ray(truth_writer_t *w, const truth_ray_t *r) {
    if (!w || !w->rays || !r) return -1;
    if (fwrite(r, sizeof(*r), 1, w->rays) != 1) return -1;
    w->ray_count++;
    return 0;
}

static void write_vec(FILE *f, const char *key, const double v[3]) {
    fprintf(f, "\"%s\":[%.9g,%.9g,%.9g]", key, v[0], v[1], v[2]);
}

int truth_writer_finish(truth_writer_t *w, const truth_header_t *h) {
    if (!w || !w->rays || !h) return -1;

    rewind(w->rays);
    const uint32_t hdr[2] = { TRUTH_RAYS_MAGIC, w->ray_count };
    if (fwrite(hdr, sizeof(hdr), 1, w->rays) != 1) return -1;
    fclose(w->rays);
    w->rays = NULL;

    char path[600];
    snprintf(path, sizeof(path), "%s.json", w->prefix);
    FILE *f = fopen(path, "wb");
    if (!f) return -1;

    fprintf(f, "{\n");
    fprintf(f, "  \"fixture\": \"%s\",\n", h->fixture);
    fprintf(f, "  \"seed\": %u,\n", h->seed);
    fprintf(f, "  \"duration_s\": %.9g,\n", h->duration_s);
    fprintf(f, "  \"scale\": %.9g,\n", h->scale);
    fprintf(f, "  \"ray_count\": %u,\n", w->ray_count);
    fprintf(f, "  \"session_origin\": {\"lat\": %.9f, \"lon\": %.9f, \"alt\": %.4f},\n",
            h->session_lat, h->session_lon, h->session_alt);

    fprintf(f, "  \"map\": {\"root_size_m\": %.9g, \"max_depth\": %d, \"coarse_depth\": %d, "
               "\"skip_near_m\": %.9g, \"queue_capacity\": %u, \"budget_per_drain\": %u, "
               "\"byte_cap\": %llu},\n",
            h->root_size_m, h->max_depth, h->coarse_depth, h->skip_near_m,
            h->queue_capacity, h->budget_per_drain, (unsigned long long)h->map_byte_cap);

    fprintf(f, "  \"vehicles\": [\n");
    for (int i = 0; i < h->vehicle_count; i++) {
        const truth_vehicle_t *v = &h->vehicles[i];
        fprintf(f, "    {\"sysid\": %u, \"origin\": {\"lat\": %.9f, \"lon\": %.9f, \"alt\": %.4f}, ",
                v->sysid, v->origin_lat, v->origin_lon, v->origin_alt);
        write_vec(f, "pos_offset_enu", v->pos_offset_enu);
        fprintf(f, ", \"note\": \"%s\"}%s\n", v->note, (i + 1 < h->vehicle_count) ? "," : "");
    }
    fprintf(f, "  ],\n");

    fprintf(f, "  \"planes\": [\n");
    for (int i = 0; i < h->scene.count; i++) {
        const geom_plane_t *p = &h->scene.planes[i];
        fprintf(f, "    {");
        write_vec(f, "point", p->point);   fprintf(f, ", ");
        write_vec(f, "normal", p->normal); fprintf(f, ", ");
        write_vec(f, "u", p->u);           fprintf(f, ", ");
        write_vec(f, "v", p->v);           fprintf(f, ", ");
        fprintf(f, "\"half_u\": %.9g, \"half_v\": %.9g, \"from_s\": %.9g, \"to_s\": %.9g, "
                   "\"label\": \"%s\"}%s\n",
                p->half_u, p->half_v, p->active_from_s, p->active_to_s, p->label,
                (i + 1 < h->scene.count) ? "," : "");
    }
    fprintf(f, "  ],\n");

    const truth_thresholds_t *t = &h->thresholds;
    fprintf(f, "  \"thresholds\": {\n");
    fprintf(f, "    \"surface_rms_max_m\": %.9g,\n", t->surface_rms_max_m);
    fprintf(f, "    \"false_occupied_max\": %.9g,\n", t->false_occupied_max);
    fprintf(f, "    \"false_free_max\": %.9g,\n", t->false_free_max);
    fprintf(f, "    \"coverage_min\": %.9g,\n", t->coverage_min);
    fprintf(f, "    \"occupied_cells_min\": %lld,\n", (long long)t->occupied_cells_min);
    fprintf(f, "    \"occupied_cells_max\": %lld,\n", (long long)t->occupied_cells_max);
    fprintf(f, "    \"memory_plateau_ratio\": %.9g,\n", t->memory_plateau_ratio);
    fprintf(f, "    \"min_rays_per_s\": %.9g,\n", t->min_rays_per_s);
    fprintf(f, "    \"require_drops\": %d,\n", t->require_drops);
    fprintf(f, "    \"contested_max_dist_m\": %.9g,\n", t->contested_max_dist_m);
    fprintf(f, "    \"require_contested\": %d,\n", t->require_contested);
    fprintf(f, "    \"contested_share_max\": %.9g,\n", t->contested_share_max);
    fprintf(f, "    \"cone_ratio_min\": %.9g,\n", t->cone_ratio_min);
    fprintf(f, "    \"weak_ratio_min\": %.9g,\n", t->weak_ratio_min);
    fprintf(f, "    \"live_nodes_max\": %lld,\n", (long long)t->live_nodes_max);
    fprintf(f, "    \"prune_blocks_min\": %lld,\n", (long long)t->prune_blocks_min);
    fprintf(f, "    \"vehicle0_rms_max\": %.9g,\n", t->vehicle0_rms_max);
    fprintf(f, "    \"vehicle1_rms_min\": %.9g,\n", t->vehicle1_rms_min);
    fprintf(f, "    \"group_false_free_max\": %.9g,\n", t->group_false_free_max);
    fprintf(f, "    \"group_min_rays\": %d\n", t->group_min_rays);
    fprintf(f, "  }\n}\n");

    fclose(f);
    return 0;
}

// ---------------------------------------------------------------- read

// A deliberately small JSON scanner: the checker only needs to find known keys
// in a file this repository writes itself.
static const char *find_key(const char *buf, const char *key) {
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(buf, pattern);
    if (!p) return NULL;
    p += strlen(pattern);
    while (*p && *p != ':') p++;
    return *p ? p + 1 : NULL;
}

static double num_after(const char *p, double fallback) {
    if (!p) return fallback;
    char *end = NULL;
    const double v = strtod(p, &end);
    return (end == p) ? fallback : v;
}

static double key_num(const char *buf, const char *key, double fallback) {
    return num_after(find_key(buf, key), fallback);
}

static void key_str(const char *buf, const char *key, char *out, size_t out_len) {
    const char *p = find_key(buf, key);
    if (!p) { if (out_len) out[0] = '\0'; return; }
    while (*p && *p != '"') p++;
    if (!*p) { if (out_len) out[0] = '\0'; return; }
    p++;
    size_t n = 0;
    while (*p && *p != '"' && n + 1 < out_len) out[n++] = *p++;
    out[n] = '\0';
}

static const char *read_vec3(const char *p, double v[3]) {
    if (!p) return NULL;
    while (*p && *p != '[') p++;
    if (!*p) return NULL;
    p++;
    for (int i = 0; i < 3; i++) {
        char *end = NULL;
        v[i] = strtod(p, &end);
        if (end == p) return NULL;
        p = end;
        while (*p && (*p == ',' || *p == ' ')) p++;
    }
    return p;
}

int truth_load(truth_t *t, const char *prefix, char *err, size_t err_len) {
    if (!t || !prefix) return -1;
    memset(t, 0, sizeof(*t));

    char path[600];
    snprintf(path, sizeof(path), "%s.json", prefix);
    FILE *f = fopen(path, "rb");
    if (!f) { if (err) snprintf(err, err_len, "cannot open %s", path); return -1; }
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    rewind(f);
    if (size <= 0) { fclose(f); if (err) snprintf(err, err_len, "empty truth header"); return -1; }
    char *buf = (char *)malloc((size_t)size + 1);
    if (!buf) { fclose(f); return -1; }
    if (fread(buf, 1, (size_t)size, f) != (size_t)size) { free(buf); fclose(f); return -1; }
    fclose(f);
    buf[size] = '\0';

    truth_header_t *h = &t->header;
    key_str(buf, "fixture", h->fixture, sizeof(h->fixture));
    h->seed = (uint32_t)key_num(buf, "seed", 0);
    h->duration_s = key_num(buf, "duration_s", 0);
    h->scale = key_num(buf, "scale", 1.0);

    const char *so = find_key(buf, "session_origin");
    h->session_lat = num_after(find_key(so ? so : buf, "lat"), 0);
    h->session_lon = num_after(find_key(so ? so : buf, "lon"), 0);
    h->session_alt = num_after(find_key(so ? so : buf, "alt"), 0);

    const char *mp = find_key(buf, "map");
    if (mp) {
        h->root_size_m      = key_num(mp, "root_size_m", 0);
        h->max_depth        = (int)key_num(mp, "max_depth", 0);
        h->coarse_depth     = (int)key_num(mp, "coarse_depth", 0);
        h->skip_near_m      = key_num(mp, "skip_near_m", 0);
        h->queue_capacity   = (uint32_t)key_num(mp, "queue_capacity", 0);
        h->budget_per_drain = (uint32_t)key_num(mp, "budget_per_drain", 0);
        h->map_byte_cap     = (size_t)key_num(mp, "byte_cap", 0);
    }

    // Vehicles.
    const char *p = strstr(buf, "\"vehicles\"");
    if (p) {
        while ((p = strstr(p, "\"sysid\"")) != NULL && h->vehicle_count < TRUTH_MAX_VEHICLES) {
            truth_vehicle_t *v = &h->vehicles[h->vehicle_count];
            // p + 7 lands on the ':' that follows "sysid", and strtod stops
            // dead on it -- so this used to return the fallback for every
            // vehicle. find_key steps past the colon and the whitespace.
            v->sysid = (uint8_t)num_after(find_key(p, "sysid"), 1);
            const char *org = strstr(p, "\"origin\"");
            if (org) {
                v->origin_lat = num_after(find_key(org, "lat"), 0);
                v->origin_lon = num_after(find_key(org, "lon"), 0);
                v->origin_alt = num_after(find_key(org, "alt"), 0);
            }
            const char *off = strstr(p, "\"pos_offset_enu\"");
            if (off) read_vec3(off, v->pos_offset_enu);
            const char *note = strstr(p, "\"note\"");
            if (note) key_str(note, "note", v->note, sizeof(v->note));
            h->vehicle_count++;
            p += 7;
            const char *next_veh = strstr(p, "\"sysid\"");
            const char *planes = strstr(buf, "\"planes\"");
            if (planes && next_veh && next_veh > planes) break;
        }
    }

    // Planes.
    p = strstr(buf, "\"planes\"");
    if (p) {
        const char *q = p;
        while ((q = strstr(q, "\"point\"")) != NULL && h->scene.count < GEOM_MAX_PLANES) {
            geom_plane_t *pl = &h->scene.planes[h->scene.count];
            memset(pl, 0, sizeof(*pl));
            read_vec3(q, pl->point);
            const char *nrm = strstr(q, "\"normal\"");
            const char *uu  = strstr(q, "\"u\"");
            const char *vv  = strstr(q, "\"v\"");
            if (nrm) read_vec3(nrm, pl->normal);
            if (uu) read_vec3(uu, pl->u);
            if (vv) read_vec3(vv, pl->v);
            pl->half_u = num_after(find_key(q, "half_u"), 0);
            pl->half_v = num_after(find_key(q, "half_v"), 0);
            pl->active_from_s = num_after(find_key(q, "from_s"), 0);
            pl->active_to_s = num_after(find_key(q, "to_s"), 1e18);
            key_str(q, "label", pl->label, sizeof(pl->label));
            h->scene.count++;
            q += 7;
        }
    }

    const char *th = strstr(buf, "\"thresholds\"");
    if (th) {
        truth_thresholds_t *x = &h->thresholds;
        x->surface_rms_max_m    = key_num(th, "surface_rms_max_m", 1e9);
        x->false_occupied_max   = key_num(th, "false_occupied_max", 1.0);
        x->false_free_max       = key_num(th, "false_free_max", 1.0);
        x->coverage_min         = key_num(th, "coverage_min", 0.0);
        x->occupied_cells_min   = (int64_t)key_num(th, "occupied_cells_min", 0);
        x->occupied_cells_max   = (int64_t)key_num(th, "occupied_cells_max", -1);
        x->memory_plateau_ratio = key_num(th, "memory_plateau_ratio", 0.0);
        x->min_rays_per_s       = key_num(th, "min_rays_per_s", 0.0);
        x->require_drops        = (int)key_num(th, "require_drops", 0);
        x->contested_max_dist_m = key_num(th, "contested_max_dist_m", 0.0);
        x->require_contested    = (int)key_num(th, "require_contested", 0);
        x->contested_share_max  = key_num(th, "contested_share_max", 0.0);
        x->cone_ratio_min       = key_num(th, "cone_ratio_min", 0.0);
        x->weak_ratio_min       = key_num(th, "weak_ratio_min", 0.0);
        x->live_nodes_max       = (int64_t)key_num(th, "live_nodes_max", 0);
        x->prune_blocks_min     = (int64_t)key_num(th, "prune_blocks_min", 0);
        x->vehicle0_rms_max     = key_num(th, "vehicle0_rms_max", 0.0);
        x->vehicle1_rms_min     = key_num(th, "vehicle1_rms_min", 0.0);
        x->group_false_free_max = key_num(th, "group_false_free_max", 0.0);
        x->group_min_rays       = (int)key_num(th, "group_min_rays", 0);
    }
    free(buf);

    // Rays.
    snprintf(path, sizeof(path), "%s.rays", prefix);
    f = fopen(path, "rb");
    if (!f) { if (err) snprintf(err, err_len, "cannot open %s", path); return -1; }
    uint32_t hdr[2] = { 0, 0 };
    if (fread(hdr, sizeof(hdr), 1, f) != 1 || hdr[0] != TRUTH_RAYS_MAGIC) {
        fclose(f);
        if (err) snprintf(err, err_len, "bad ray table magic");
        return -1;
    }
    t->ray_count = hdr[1];
    if (t->ray_count) {
        t->rays = (truth_ray_t *)malloc((size_t)t->ray_count * sizeof(truth_ray_t));
        if (!t->rays) { fclose(f); return -1; }
        if (fread(t->rays, sizeof(truth_ray_t), t->ray_count, f) != t->ray_count) {
            free(t->rays); t->rays = NULL; fclose(f);
            if (err) snprintf(err, err_len, "short ray table");
            return -1;
        }
    }
    fclose(f);
    return 0;
}

void truth_free(truth_t *t) {
    if (!t) return;
    free(t->rays);
    t->rays = NULL;
    t->ray_count = 0;
}
