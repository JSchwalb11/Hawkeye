// Scored checker for the synthetic ray fixtures.
//
// Replays a recorded tlog through the real ingest path -- the same decode, the
// same transforms, the same bounded queue the viewer uses -- and scores the
// resulting map against the ground truth the injector published. Every number
// here is asserted in CI, so a map change that improves the picture but
// regresses false-occupied shows up as a number rather than an argument.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#include <mavlink.h>

#include "map_session.h"
#include "mavlink_map_decode.h"
#include "canvas.h"
#include "ortho_render.h"
#include "tlog.h"
#include "truth.h"

#define FRAME_NS (int64_t)(1000000000LL / 60)

// Distance from a cell to the surface is measured from the cell's nearest face,
// not its centre, so a coarse cone-widened cell is not punished for being big.
#define SURFACE_SLACK_M 0.60

typedef struct {
    const octomap_t *map;
    const truth_t   *t;
    double           final_t_s;

    uint64_t occupied_cells, free_cells, unknown_cells;
    double   err_sq_sum;
    uint64_t err_n;
    double   err_max;
    uint64_t false_occupied;

    uint64_t contested_cells;
    double   contested_max_dist;

    uint64_t vanished_occupied;

    double   veh_size_sum[TRUTH_MAX_VEHICLES];
    uint64_t veh_size_n[TRUTH_MAX_VEHICLES];
    double   veh_lo_sum[TRUTH_MAX_VEHICLES];
    uint64_t veh_lo_n[TRUTH_MAX_VEHICLES];
    double   veh_err_sq[TRUTH_MAX_VEHICLES];
    uint64_t veh_err_n[TRUTH_MAX_VEHICLES];
} scan_t;

static bool scene_has_active(const geom_scene_t *s, double t_s) {
    for (int i = 0; i < s->count; i++) if (geom_plane_active(&s->planes[i], t_s)) return true;
    return false;
}

// Distance to the nearest patch that is no longer active -- used to prove the
// `vanishing` fixture really cleared rather than merely stopped being updated.
static double distance_to_vanished(const geom_scene_t *s, double t_s, const double p[3]) {
    geom_scene_t gone;
    memset(&gone, 0, sizeof(gone));
    for (int i = 0; i < s->count; i++) {
        if (geom_plane_active(&s->planes[i], t_s)) continue;
        if (s->planes[i].active_to_s > t_s) continue;   // not yet born, not removed
        gone.planes[gone.count++] = s->planes[i];
    }
    if (gone.count == 0) return 1e18;
    // Every patch in `gone` is inactive, so ask at a time when they all were.
    return geom_scene_distance(&gone, gone.planes[0].active_from_s, p);
}

static void scan_leaf(const om_leaf_t *leaf, void *user) {
    scan_t *s = (scan_t *)user;

    if (leaf->state == OM_OCCUPIED) s->occupied_cells++;
    else if (leaf->state == OM_FREE) s->free_cells++;
    else { s->unknown_cells++; return; }

    if (leaf->state != OM_OCCUPIED) return;

    const double d = geom_scene_distance(&s->t->header.scene, s->final_t_s, leaf->center);
    double err = d - leaf->size * 0.5;
    if (err < 0.0) err = 0.0;
    if (d < 1e17) {
        s->err_sq_sum += err * err;
        s->err_n++;
        if (err > s->err_max) s->err_max = err;
        if (err > SURFACE_SLACK_M) s->false_occupied++;
    } else {
        // No geometry at all: every occupied cell is a false positive.
        s->false_occupied++;
    }

    const double dv = distance_to_vanished(&s->t->header.scene, s->final_t_s, leaf->center);
    if (dv < 1e17 && dv - leaf->size * 0.5 <= SURFACE_SLACK_M && err > SURFACE_SLACK_M)
        s->vanished_occupied++;

    if (om_node_contested(s->map, leaf->node)) {
        s->contested_cells++;
        if (d < 1e17 && d > s->contested_max_dist) s->contested_max_dist = d;
    }

    // Attribute cells seen by exactly one vehicle, so the cone and weak
    // fixtures can compare like with like.
    for (int v = 0; v < s->t->header.vehicle_count && v < TRUTH_MAX_VEHICLES; v++) {
        if (leaf->node->observers == om_vehicle_bit((uint8_t)v)) {
            s->veh_size_sum[v] += leaf->size;
            s->veh_size_n[v]++;
            s->veh_lo_sum[v] += fabs((double)leaf->node->log_odds);
            s->veh_lo_n[v]++;
            if (d < 1e17) { s->veh_err_sq[v] += err * err; s->veh_err_n[v]++; }
            break;
        }
    }
}

static double wall_now(void) {
    struct timespec ts;
#ifdef CLOCK_MONOTONIC
    clock_gettime(CLOCK_MONOTONIC, &ts);
#else
    timespec_get(&ts, TIME_UTC);
#endif
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

typedef struct {
    double   surface_rms;
    double   surface_max;
    double   false_occupied_rate;
    double   false_free_rate;
    double   coverage;
    uint64_t occupied_cells;
    uint64_t free_cells;
    uint64_t contested_cells;
    double   contested_max_dist;
    uint64_t vanished_occupied;
    size_t   peak_bytes, half_bytes, end_bytes;
    uint32_t half_nodes, end_nodes;
    uint32_t nodes_before_prune, nodes_after_prune;
    uint32_t blocks_reclaimed;
    double   rays_per_s;
    uint64_t rays_inserted, rays_dropped;
    uint64_t drop_events;
    double   cone_ratio, weak_ratio;
    double   veh_mean_size[TRUTH_MAX_VEHICLES];
    double   veh_mean_lo[TRUTH_MAX_VEHICLES];
    double   veh_rms[TRUTH_MAX_VEHICLES];
    uint64_t veh_cells[TRUTH_MAX_VEHICLES];
} report_t;

static void configure_session(map_session_t *ms, const truth_t *t) {
    map_session_config_t cfg;
    map_session_config_defaults(&cfg);
    cfg.root_size_m = t->header.root_size_m;
    cfg.max_depth = t->header.max_depth;
    cfg.coarse_depth = t->header.coarse_depth;
    cfg.skip_near_m = t->header.skip_near_m;
    cfg.queue_capacity = t->header.queue_capacity;
    cfg.budget_per_drain = t->header.budget_per_drain;
    cfg.map_byte_cap = t->header.map_byte_cap;
    cfg.max_vehicles = t->header.vehicle_count > 0 ? t->header.vehicle_count : 1;
    cfg.origin_policy = FLEET_ORIGIN_FIRST_SEEN;
    if (map_session_init(ms, &cfg) != 0) {
        fprintf(stderr, "map session init failed\n");
        exit(1);
    }
    // The fixture publishes its session origin; pinning it means the checker
    // scores in exactly the frame the truth was written in.
    fleet_frame_set_explicit(&ms->frame, t->header.session_lat,
                             t->header.session_lon, t->header.session_alt);
}

static uint64_t count_drop_events(const timeline_t *tl) {
    uint64_t n = 0;
    const uint32_t start = (tl->event_head - tl->event_count) & (tl->event_cap - 1);
    for (uint32_t i = 0; i < tl->event_count; i++) {
        const tl_event_t *e = &tl->events[(start + i) & (tl->event_cap - 1)];
        if (e->kind == TL_EVENT_MAP_DROP) n++;
    }
    return n;
}

static int replay_tlog(map_session_t *ms, const char *tlog_path, const truth_t *t,
                       report_t *rep) {
    tlog_reader_t r;
    if (tlog_reader_open(&r, tlog_path, 0) != 0) {
        fprintf(stderr, "cannot open tlog %s\n", tlog_path);
        return -1;
    }

    int64_t first_ns = 0, last_ns = 0;
    if (!tlog_reader_span(&r, &first_ns, &last_ns)) {
        fprintf(stderr, "tlog has no frames\n");
        tlog_reader_close(&r);
        return -1;
    }
    tlog_reader_rewind(&r);

    const int64_t half_ns = first_ns + (last_ns - first_ns) / 2;
    int64_t next_drain = first_ns + FRAME_NS;
    bool half_sampled = false;

    const double t0 = wall_now();
    mavlink_message_t msg;
    int64_t arrival = 0;
    while (tlog_reader_next(&r, (struct __mavlink_message *)&msg, &arrival) == 1) {
        while (arrival >= next_drain) {
            map_ingest_drain(&ms->ingest, &ms->map, 1.0f / 60.0f);
            next_drain += FRAME_NS;
        }
        if (!half_sampled && arrival >= half_ns) {
            // Sampled mid-run so `endurance` can show memory plateauing rather
            // than climbing.
            octomap_prune(&ms->map);
            rep->half_bytes = octomap_bytes(&ms->map);
            rep->half_nodes = ms->map.node_count - ms->map.free_blocks * 8;
            half_sampled = true;
        }
        mavlink_map_decode(ms, -1, (struct __mavlink_message *)&msg, arrival);
    }
    map_ingest_drain_all(&ms->ingest, &ms->map);
    const double elapsed = wall_now() - t0;

    tlog_reader_close(&r);

    rep->rays_inserted = ms->ingest.stats.inserted;
    rep->rays_dropped = ms->ingest.stats.dropped;
    rep->drop_events = count_drop_events(&ms->timeline);
    rep->rays_per_s = elapsed > 0.0 ? (double)rep->rays_inserted / elapsed : 0.0;
    rep->peak_bytes = ms->map.stats.peak_bytes;
    rep->end_bytes = octomap_bytes(&ms->map);
    (void)t;
    return 0;
}

static void measure(map_session_t *ms, const truth_t *t, report_t *rep) {
    scan_t s;
    memset(&s, 0, sizeof(s));
    s.map = &ms->map;
    s.t = t;
    s.final_t_s = t->header.duration_s;

    octomap_iterate(&ms->map, scan_leaf, &s);

    rep->occupied_cells = s.occupied_cells;
    rep->free_cells = s.free_cells;
    rep->contested_cells = s.contested_cells;
    rep->contested_max_dist = s.contested_max_dist;
    rep->vanished_occupied = s.vanished_occupied;
    rep->surface_rms = s.err_n ? sqrt(s.err_sq_sum / (double)s.err_n) : 0.0;
    rep->surface_max = s.err_max;
    rep->false_occupied_rate = s.occupied_cells
        ? (double)s.false_occupied / (double)s.occupied_cells : 0.0;

    for (int v = 0; v < t->header.vehicle_count && v < TRUTH_MAX_VEHICLES; v++) {
        rep->veh_cells[v] = s.veh_size_n[v];
        rep->veh_mean_size[v] = s.veh_size_n[v] ? s.veh_size_sum[v] / (double)s.veh_size_n[v] : 0.0;
        rep->veh_mean_lo[v] = s.veh_lo_n[v] ? s.veh_lo_sum[v] / (double)s.veh_lo_n[v] : 0.0;
        rep->veh_rms[v] = s.veh_err_n[v] ? sqrt(s.veh_err_sq[v] / (double)s.veh_err_n[v]) : 0.0;
    }
    if (t->header.vehicle_count >= 2 && rep->veh_mean_size[1] > 0.0)
        rep->cone_ratio = rep->veh_mean_size[0] / rep->veh_mean_size[1];
    if (t->header.vehicle_count >= 2 && rep->veh_mean_lo[1] > 0.0)
        rep->weak_ratio = rep->veh_mean_lo[0] / rep->veh_mean_lo[1];

    // False-free and coverage, both scored against the rays the injector says
    // it actually cast. Nothing is re-derived here.
    const double leaf = octomap_cell_size(&ms->map, ms->map.max_depth);
    uint64_t surf_free = 0, surf_occ = 0;
    uint64_t swept = 0, observed = 0;

    const uint32_t stride = (t->ray_count > 200000u) ? (t->ray_count / 200000u) : 1u;
    for (uint32_t i = 0; i < t->ray_count; i += stride) {
        const truth_ray_t *ray = &t->rays[i];
        const double end[3] = { ray->endpoint[0], ray->endpoint[1], ray->endpoint[2] };

        const double o[3] = { ray->origin[0], ray->origin[1], ray->origin[2] };
        double d[3] = { end[0] - o[0], end[1] - o[1], end[2] - o[2] };
        const double len = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if (len < 1e-6) continue;
        for (int k = 0; k < 3; k++) d[k] /= len;

        if (ray->hit) {
            const double dist = geom_scene_distance(&t->header.scene, s.final_t_s, end);
            if (dist <= leaf * 2.0) {
                // A surface counts as represented when the map holds occupancy
                // within one cell of it. Which side of a cell boundary the hit
                // lands on is quantisation at the map's own resolution, not
                // erosion; losing the surface entirely is what this measures.
                bool represented = false, any_free = false;
                for (int a = -1; a <= 1 && !represented; a++)
                for (int b = -1; b <= 1 && !represented; b++)
                for (int c = -1; c <= 1 && !represented; c++) {
                    const double p[3] = { end[0] + a * leaf,
                                          end[1] + b * leaf,
                                          end[2] + c * leaf };
                    const om_state_t st = octomap_query(&ms->map, p[0], p[1], p[2]);
                    if (st == OM_OCCUPIED) represented = true;
                    else if (st == OM_FREE) any_free = true;
                }
                if (represented) surf_occ++;
                else if (any_free) surf_free++;
            }
        }

        // Coverage: the volume this ray swept should not still be unknown.
        for (double u = t->header.skip_near_m + 0.5; u < len; u += 1.0) {
            const double p[3] = { o[0] + d[0] * u, o[1] + d[1] * u, o[2] + d[2] * u };
            swept++;
            if (octomap_query(&ms->map, p[0], p[1], p[2]) != OM_UNKNOWN) observed++;
        }
    }
    rep->false_free_rate = (surf_free + surf_occ)
        ? (double)surf_free / (double)(surf_free + surf_occ) : 0.0;
    rep->coverage = swept ? (double)observed / (double)swept : 0.0;

    // One final prune, reported. Most of the collapsing already happened on the
    // ingest layer's own schedule, so a small number here is the healthy case
    // -- it means memory was being reclaimed all along rather than at the end.
    rep->nodes_before_prune = ms->map.node_count - ms->map.free_blocks * 8;
    rep->blocks_reclaimed = octomap_prune(&ms->map);
    rep->nodes_after_prune = ms->map.node_count - ms->map.free_blocks * 8;
    rep->end_nodes = rep->nodes_after_prune;
}

static int fail(const char *what, double got, const char *cmp, double want) {
    printf("  FAIL  %-24s %.6g %s %.6g\n", what, got, cmp, want);
    return 1;
}

static int assert_thresholds(const truth_t *t, const report_t *r) {
    const truth_thresholds_t *th = &t->header.thresholds;
    int bad = 0;

    if (scene_has_active(&t->header.scene, t->header.duration_s)) {
        if (th->surface_rms_max_m > 0.0 && r->surface_rms > th->surface_rms_max_m)
            bad += fail("surface RMS (m)", r->surface_rms, ">", th->surface_rms_max_m);
        if (r->false_occupied_rate > th->false_occupied_max)
            bad += fail("false-occupied rate", r->false_occupied_rate, ">", th->false_occupied_max);
        if (r->false_free_rate > th->false_free_max)
            bad += fail("false-free rate", r->false_free_rate, ">", th->false_free_max);
    }

    if (th->coverage_min > 0.0 && r->coverage < th->coverage_min)
        bad += fail("coverage", r->coverage, "<", th->coverage_min);

    if ((int64_t)r->occupied_cells < th->occupied_cells_min)
        bad += fail("occupied cells", (double)r->occupied_cells, "<",
                    (double)th->occupied_cells_min);
    if (th->occupied_cells_max >= 0 && (int64_t)r->occupied_cells > th->occupied_cells_max)
        bad += fail("occupied cells", (double)r->occupied_cells, ">",
                    (double)th->occupied_cells_max);

    // Anything the geometry removed must actually have been carved away. A
    // hits-only map can never pass this.
    if (r->vanished_occupied > 0)
        bad += fail("cells on removed geometry", (double)r->vanished_occupied, ">", 0.0);

    if (th->require_contested) {
        if (r->contested_cells == 0)
            bad += fail("contested cells", 0.0, "<", 1.0);
        else if (th->contested_max_dist_m > 0.0 && r->contested_max_dist > th->contested_max_dist_m)
            bad += fail("contested spread (m)", r->contested_max_dist, ">",
                        th->contested_max_dist_m);
    }

    if (th->cone_ratio_min > 0.0 && r->cone_ratio < th->cone_ratio_min)
        bad += fail("cone size ratio", r->cone_ratio, "<", th->cone_ratio_min);

    if (th->weak_ratio_min > 0.0 && r->weak_ratio < th->weak_ratio_min)
        bad += fail("clean/weak log-odds", r->weak_ratio, "<", th->weak_ratio_min);

    // The clock pair. Vehicle 0 must be clean; vehicle 1 must be visibly worse,
    // because a viewer that ignored ranging timestamps would smear both alike.
    if (th->vehicle0_rms_max > 0.0 && r->veh_rms[0] > th->vehicle0_rms_max)
        bad += fail("vehicle 0 surface RMS", r->veh_rms[0], ">", th->vehicle0_rms_max);
    if (th->vehicle1_rms_min > 0.0 && r->veh_rms[1] < th->vehicle1_rms_min)
        bad += fail("vehicle 1 surface RMS", r->veh_rms[1], "<", th->vehicle1_rms_min);

    if (th->min_rays_per_s > 0.0 && r->rays_per_s < th->min_rays_per_s)
        bad += fail("sustained rays/s", r->rays_per_s, "<", th->min_rays_per_s);

    if (th->require_drops) {
        if (r->rays_dropped == 0)
            bad += fail("rays dropped", 0.0, "<", 1.0);
        // Dropped is not enough: the drop has to be reported, not absorbed.
        if (r->rays_dropped > 0 && r->drop_events == 0)
            bad += fail("drop events on timeline", 0.0, "<", 1.0);
    }

    // Memory must plateau, not climb. Live node count is the honest measure:
    // the byte figure only moves when the pool doubles, so a map that grows
    // steadily could sit at the same byte total for a long while.
    if (th->memory_plateau_ratio > 0.0 && r->half_nodes > 0) {
        const double ratio = (double)r->end_nodes / (double)r->half_nodes;
        if (ratio > th->memory_plateau_ratio)
            bad += fail("live nodes late/early", ratio, ">", th->memory_plateau_ratio);
        if (r->end_bytes > 0 && r->half_bytes > 0) {
            const double byte_ratio = (double)r->end_bytes / (double)r->half_bytes;
            if (byte_ratio > th->memory_plateau_ratio + 1.0)
                bad += fail("memory late/early", byte_ratio, ">",
                            th->memory_plateau_ratio + 1.0);
        }
    }

    return bad;
}

static void print_report(const truth_t *t, const report_t *r) {
    printf("fixture: %s  (%d vehicle%s, %.1f s, %u truth rays)\n",
           t->header.fixture, t->header.vehicle_count,
           t->header.vehicle_count == 1 ? "" : "s", t->header.duration_s, t->ray_count);
    printf("  surface RMS            %.4f m  (max %.4f m)\n", r->surface_rms, r->surface_max);
    printf("  false-occupied rate    %.5f\n", r->false_occupied_rate);
    printf("  false-free rate        %.5f\n", r->false_free_rate);
    printf("  coverage completeness  %.5f\n", r->coverage);
    printf("  cells occupied/free    %llu / %llu\n",
           (unsigned long long)r->occupied_cells, (unsigned long long)r->free_cells);
    printf("  contested cells        %llu  (max %.2f m from surface)\n",
           (unsigned long long)r->contested_cells, r->contested_max_dist);
    printf("  cells on removed geom  %llu\n", (unsigned long long)r->vanished_occupied);
    printf("  memory peak/half/end   %.2f / %.2f / %.2f MiB\n",
           r->peak_bytes / 1048576.0, r->half_bytes / 1048576.0, r->end_bytes / 1048576.0);
    printf("  nodes before/after     %u / %u  (%u blocks reclaimed)\n",
           r->nodes_before_prune, r->nodes_after_prune, r->blocks_reclaimed);
    printf("  live nodes half/end    %u / %u\n", r->half_nodes, r->end_nodes);
    printf("  rays inserted/dropped  %llu / %llu  (%llu drop events)\n",
           (unsigned long long)r->rays_inserted, (unsigned long long)r->rays_dropped,
           (unsigned long long)r->drop_events);
    printf("  sustained rays/s       %.0f\n", r->rays_per_s);
    for (int v = 0; v < t->header.vehicle_count && v < TRUTH_MAX_VEHICLES; v++)
        printf("  vehicle %d exclusive    %llu cells, mean size %.3f m, "
               "mean |log-odds| %.1f, RMS %.4f m\n",
               v, (unsigned long long)r->veh_cells[v], r->veh_mean_size[v],
               r->veh_mean_lo[v], r->veh_rms[v]);
    if (r->cone_ratio > 0.0) printf("  cone size ratio        %.3f\n", r->cone_ratio);
    if (r->weak_ratio > 0.0) printf("  clean/weak log-odds    %.3f\n", r->weak_ratio);
}

#ifndef _WIN32
// Live listen mode: the same map session, fed from a real socket. Proves the
// fixtures reach the map over the wire and not through a side door.
//
// The socket is bound before the injector is spawned, so no frame is
// transmitted into a closed port and the test does not depend on timing luck.
static int run_listen(const truth_t *t, int port, double seconds,
                      const char *spawn_path, const char *spawn_fixture) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons((uint16_t)port);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }

    pid_t child = -1;
    if (spawn_path && spawn_fixture) {
        char target[64];
        snprintf(target, sizeof(target), "127.0.0.1:%d", port);
        child = fork();
        if (child == 0) {
            execl(spawn_path, spawn_path, "--fixture", spawn_fixture,
                  "--udp", target, "--realtime", (char *)NULL);
            _exit(127);
        }
        if (child < 0) { perror("fork"); close(sock); return 1; }
    }

    struct timeval tv = { 1, 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    map_session_t ms;
    configure_session(&ms, t);

    const double t0 = wall_now();
    double last_rx = t0;
    uint8_t buf[2048];
    mavlink_message_t msg;
    mavlink_status_t status;
    uint64_t frames = 0;

    while (wall_now() - t0 < seconds) {
        const ssize_t n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        if (n > 0) {
            last_rx = wall_now();
            for (ssize_t i = 0; i < n; i++)
                if (mavlink_parse_char(0, buf[i], &msg, &status)) {
                    mavlink_map_decode(&ms, -1, (struct __mavlink_message *)&msg,
                                       (int64_t)(wall_now() * 1e9));
                    frames++;
                }
        } else if (frames > 0 && wall_now() - last_rx > 2.0) {
            break;   // the injector finished
        }
        map_ingest_drain(&ms.ingest, &ms.map, 1.0f / 60.0f);
    }
    map_ingest_drain_all(&ms.ingest, &ms.map);
    close(sock);
    if (child > 0) {
        int status = 0;
        kill(child, SIGTERM);
        waitpid(child, &status, 0);
    }

    report_t rep;
    memset(&rep, 0, sizeof(rep));
    measure(&ms, t, &rep);
    printf("wire smoke: %llu frames, %llu occupied cells, surface RMS %.4f m\n",
           (unsigned long long)frames, (unsigned long long)rep.occupied_cells, rep.surface_rms);

    int bad = 0;
    if (frames == 0) { printf("  FAIL  no frames arrived over UDP\n"); bad++; }
    if (rep.occupied_cells == 0) { printf("  FAIL  wire path produced no occupied cells\n"); bad++; }
    if (rep.surface_rms > t->header.thresholds.surface_rms_max_m * 2.0) {
        printf("  FAIL  wire path surface RMS %.4f m\n", rep.surface_rms);
        bad++;
    }
    map_session_free(&ms);
    return bad ? 1 : 0;
}
#endif

// ---------------------------------------------------------------- rendering

#define SHEET_W 1360
#define SHEET_H 980
#define COL_BG    0x080A0Eu
#define COL_HEAD  0xE6EDF3u
#define COL_DIM   0x8B98A5u
#define COL_OK    0x7BE495u
#define COL_WARN  0xFFD166u

static void legend_row(canvas_t *c, int x, int *y, uint32_t swatch, const char *text) {
    canvas_fill_rect(c, x, *y, 9, 9, swatch, 1.0f);
    canvas_rect_outline(c, x, *y, 9, 9, 0x2A3341u);
    canvas_text(c, x + 15, *y + 1, text, COL_DIM, 1);
    *y += 15;
}

// One sheet per fixture: the map as built, in the four draw modes, beside the
// numbers it was scored on. Rendered from the octree rather than screenshotted,
// so it reproduces on a machine with no GPU.
static int render_sheet(const map_session_t *ms, const truth_t *t, const report_t *r,
                        const char *path, int focus) {
    canvas_t c;
    if (canvas_init(&c, SHEET_W, SHEET_H, COL_BG) != 0) return -1;

    char line[160];
    snprintf(line, sizeof(line), "HAWKEYE FLEET MAP - FIXTURE %s", t->header.fixture);
    canvas_text(&c, 16, 14, line, COL_HEAD, 2);

    snprintf(line, sizeof(line), "%d VEHICLE(S)  %.0f S  %llu RAYS INSERTED  "
                                "%llu DROPPED  %.0f RAYS/S",
             t->header.vehicle_count, t->header.duration_s,
             (unsigned long long)r->rays_inserted, (unsigned long long)r->rays_dropped,
             r->rays_per_s);
    canvas_text(&c, 16, 38, line, COL_DIM, 1);

    snprintf(line, sizeof(line), "SURFACE RMS %.3f M   FALSE-OCC %.4f   FALSE-FREE %.4f   "
                                "COVERAGE %.4f   OCCUPIED %llu   CONTESTED %llu",
             r->surface_rms, r->false_occupied_rate, r->false_free_rate, r->coverage,
             (unsigned long long)r->occupied_cells, (unsigned long long)r->contested_cells);
    canvas_text(&c, 16, 52, line, COL_DIM, 1);

    double mn[3], mx[3];
    ortho_auto_bounds(&ms->map, 3.0, mn, mx);

    ortho_opts_t o;
    memset(&o, 0, sizeof(o));
    memcpy(o.min, mn, sizeof(mn));
    memcpy(o.max, mx, sizeof(mx));
    o.focus_mask = om_vehicle_bit((uint8_t)(focus < 0 ? 0 : focus));
    o.truth = &t->header.scene;
    o.truth_time_s = t->header.duration_s;
    o.tracks = &ms->timeline;
    o.track_count = t->header.vehicle_count;

    const int pad = 12;
    const int top = 72;
    const int pw = (SHEET_W - pad * 4) / 3;
    const int ph = (SHEET_H - top - pad * 3) / 2;

    ortho_draw_panel(&c, pad, top, pw, ph, &ms->map, ORTHO_TOP, ORTHO_OCCUPANCY, &o,
                     "OCCUPANCY");
    ortho_draw_panel(&c, pad * 2 + pw, top, pw, ph, &ms->map, ORTHO_SIDE, ORTHO_OCCUPANCY, &o,
                     "OCCUPANCY");
    ortho_draw_panel(&c, pad * 3 + pw * 2, top, pw, ph, &ms->map, ORTHO_TOP, ORTHO_COVERAGE, &o,
                     "COVERAGE");
    ortho_draw_panel(&c, pad, top + ph + pad, pw, ph, &ms->map, ORTHO_TOP, ORTHO_DIVERGENCE, &o,
                     "DIVERGENCE");
    snprintf(line, sizeof(line), "CONTRIBUTION V%d", focus < 0 ? 0 : focus);
    ortho_draw_panel(&c, pad * 2 + pw, top + ph + pad, pw, ph, &ms->map, ORTHO_TOP,
                     ORTHO_CONTRIBUTION, &o, line);

    // Reference panel: legend, time alignment, memory, per-vehicle figures.
    const int rx = pad * 3 + pw * 2, ry = top + ph + pad;
    canvas_fill_rect(&c, rx, ry, pw, ph, 0x0E1116u, 1.0f);
    canvas_rect_outline(&c, rx, ry, pw, ph, 0x2A3341u);
    canvas_text(&c, rx + 6, ry + 5, "KEY AND FIGURES", COL_HEAD, 1);

    int y = ry + 24;
    legend_row(&c, rx + 8, &y, 0xFFB454u, "OCCUPIED (LOG-ODDS -> OPACITY)");
    legend_row(&c, rx + 8, &y, 0x2E6F8Eu, "FREE (CARVED)");
    legend_row(&c, rx + 8, &y, 0x0E1116u, "UNKNOWN (NEVER OBSERVED)");
    legend_row(&c, rx + 8, &y, 0x35C4A0u, "OBSERVED - COVERAGE VIEW");
    legend_row(&c, rx + 8, &y, 0xFF4FA3u, "CONTESTED - FLEET DISAGREES");
    legend_row(&c, rx + 8, &y, 0x4FC3FFu, "FOCUSED VEHICLE ONLY");
    legend_row(&c, rx + 8, &y, 0xF0F6FCu, "SEEN BY FOCUS AND OTHERS / TRUE SURFACE");

    y += 6;
    canvas_text(&c, rx + 8, y, "TIME ALIGNMENT", COL_HEAD, 1); y += 14;
    for (int v = 0; v < t->header.vehicle_count && v < 6; v++) {
        const map_vehicle_t *mv = &ms->veh[v];
        snprintf(line, sizeof(line), "V%d %-12s OFF %+.3f S", v,
                 timebase_provenance_name(mv->tb.provenance),
                 (double)mv->tb.offset_ns * 1e-9);
        canvas_fill_rect(&c, rx + 8, y + 1, 7, 7, ortho_vehicle_colour(v), 1.0f);
        canvas_text(&c, rx + 20, y, line, COL_DIM, 1);
        y += 13;
    }
    snprintf(line, sizeof(line), "FLEET SPREAD %.3f S",
             (double)map_session_time_spread_ns(ms) * 1e-9);
    canvas_text(&c, rx + 8, y, line, COL_DIM, 1); y += 18;

    canvas_text(&c, rx + 8, y, "MEMORY AND PRUNING", COL_HEAD, 1); y += 14;
    snprintf(line, sizeof(line), "LIVE NODES %u -> %u", r->half_nodes, r->end_nodes);
    canvas_text(&c, rx + 8, y, line, COL_DIM, 1); y += 13;
    snprintf(line, sizeof(line), "PEAK %.2f MIB  END %.2f MIB",
             r->peak_bytes / 1048576.0, r->end_bytes / 1048576.0);
    canvas_text(&c, rx + 8, y, line, COL_DIM, 1); y += 18;

    canvas_text(&c, rx + 8, y, "PER-VEHICLE SURFACE RMS", COL_HEAD, 1); y += 14;
    for (int v = 0; v < t->header.vehicle_count && v < 6; v++) {
        snprintf(line, sizeof(line), "V%d %llu CELLS  RMS %.3f M", v,
                 (unsigned long long)r->veh_cells[v], r->veh_rms[v]);
        canvas_fill_rect(&c, rx + 8, y + 1, 7, 7, ortho_vehicle_colour(v), 1.0f);
        canvas_text(&c, rx + 20, y, line, COL_DIM, 1);
        y += 13;
    }

    const int rc = canvas_write_png(&c, path);
    canvas_free(&c);
    return rc;
}

static void usage(void) {
    printf("map_checker --truth <prefix> [--tlog <path>] [--render <out.png>] [--focus <n>]\n");
    printf("            [--listen <port> [--spawn <injector> --fixture <name>] --seconds <s>]\n");
}

int main(int argc, char **argv) {
    const char *truth_prefix = NULL;
    const char *tlog_path = NULL;
    const char *spawn_path = NULL;
    const char *spawn_fixture = NULL;
    const char *render_path = NULL;
    int listen_port = 0;
    int focus = 0;
    double seconds = 30.0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--truth") == 0 && i + 1 < argc) truth_prefix = argv[++i];
        else if (strcmp(argv[i], "--tlog") == 0 && i + 1 < argc) tlog_path = argv[++i];
        else if (strcmp(argv[i], "--listen") == 0 && i + 1 < argc) listen_port = atoi(argv[++i]);
        else if (strcmp(argv[i], "--spawn") == 0 && i + 1 < argc) spawn_path = argv[++i];
        else if (strcmp(argv[i], "--fixture") == 0 && i + 1 < argc) spawn_fixture = argv[++i];
        else if (strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) seconds = atof(argv[++i]);
        else if (strcmp(argv[i], "--render") == 0 && i + 1 < argc) render_path = argv[++i];
        else if (strcmp(argv[i], "--focus") == 0 && i + 1 < argc) focus = atoi(argv[++i]);
        else { usage(); return 2; }
    }
    if (!truth_prefix || (!tlog_path && !listen_port)) { usage(); return 2; }

    truth_t truth;
    char err[256] = {0};
    if (truth_load(&truth, truth_prefix, err, sizeof(err)) != 0) {
        fprintf(stderr, "truth load failed: %s\n", err);
        return 1;
    }

#ifndef _WIN32
    if (listen_port) {
        const int rc = run_listen(&truth, listen_port, seconds, spawn_path, spawn_fixture);
        truth_free(&truth);
        return rc;
    }
#endif

    map_session_t ms;
    configure_session(&ms, &truth);

    report_t rep;
    memset(&rep, 0, sizeof(rep));
    if (replay_tlog(&ms, tlog_path, &truth, &rep) != 0) {
        map_session_free(&ms);
        truth_free(&truth);
        return 1;
    }
    measure(&ms, &truth, &rep);
    print_report(&truth, &rep);

    if (render_path) {
        if (render_sheet(&ms, &truth, &rep, render_path, focus) != 0)
            fprintf(stderr, "warning: could not write %s\n", render_path);
        else
            printf("  rendered %s\n", render_path);
    }

    const int bad = assert_thresholds(&truth, &rep);
    if (bad == 0) printf("  PASS  all thresholds met\n");

    map_session_free(&ms);
    truth_free(&truth);
    return bad ? 1 : 0;
}
