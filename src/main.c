#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#include <direct.h>
#endif
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "raylib.h"
#include "raymath.h"
#include "data_source.h"
#include "vehicle.h"
#include "scene.h"
#include "hud.h"
#include "ui_logic.h"
#include "debug_panel.h"
#include "ortho_panel.h"
#include "theme.h"
#include "asset_path.h"
#include "replay_conflict.h"
#include "replay_trail.h"
#include "replay_markers.h"
#include "ui_marker_input.h"
#include "tactical_hud.h"
#include "map_session.h"
#include "map_render.h"
#include "capture.h"
#include "map_hud.h"
#include "quality_overlay.h"
#include "skynet_manifest.h"

#define VEHICLE_SANITY_LIMIT 255
// Selector paging is shared with the numpad renderer; see hud.h.
#define FLEET_PAGE_SIZE HUD_FLEET_PAGE_SIZE
#define EARTH_RADIUS 6371000.0

#include "correlation.h"

#define CHORD_TIMEOUT_S 0.3
#define CLICK_DRAG_SLOP_PX 4.0f

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("  -udp <port>    UDP base port (default: 19410); vehicle i uses base + i\n");
    printf("  -n <count>     Number of vehicles (default: 1, sanity limit: %d)\n", VEHICLE_SANITY_LIMIT);
    printf("  -mc            Multicopter model (default)\n");
    printf("  -fw            Fixed-wing model\n");
    printf("  -ts            Tailsitter model\n");
    printf("  -origin <lat> <lon> <alt>  NED origin in degrees/meters (default: PX4 SIH)\n");
    printf("  --replay <file1.ulg> [file2.ulg ...]  Replay ULog file(s)\n");
    printf("  --ghost <file1.ulg> [file2.ulg ...]   Ghost mode replay\n");
    printf("  -w <width>     Window width (default: 1280)\n");
    printf("  -h <height>    Window height (default: 720)\n");
    printf("\n  Fleet map (shared occupancy map built from ranging messages):\n");
    printf("  --tlog <f.tlog> [...]  Replay tlog(s): raw frames with arrival times\n");
    printf("  --bin <f.bin> [...]    Replay ArduPilot DataFlash log(s)\n");
    printf("  --run <run.json>       Open a skynet run record (a manifest of logs)\n");
    printf("  --record <out.tlog>    Record live MAVLink to a tlog as the viewer saw it\n");
    printf("  --no-map               Do not build the shared map\n");
    printf("  --map-cap <MiB>        Map memory ceiling (default: 256)\n");
    printf("  --map-res <m>          Leaf size in metres (default: 0.25)\n");
    printf("  --map-origin centroid  Use the fleet centroid rather than the first origin\n");
    printf("  --view <mode>          Start in a fullscreen ortho view: top, bottom,\n");
    printf("                         front, back, left, right (chase = default)\n");
    printf("  --view-span <m>        Ortho span in metres (default: fit to the map)\n");
    printf("  --follow-map           Aim the camera at the map, not the aircraft\n");
}

/* Thin wrapper: delegates to the testable inline in ui_logic.h */
static void apply_vehicle_selection_hud(hud_t *hud, int idx, bool pin,
                                        int *selected, int vehicle_count) {
    apply_vehicle_selection(hud->pinned, &hud->pinned_count,
                            idx, pin, selected, vehicle_count);
}



static void draw_edge_indicators(const vehicle_t *vehicles, int vehicle_count,
                                  int selected, Camera3D camera, Font font,
                                  float scale)
{
    (void)scale;
    int ei_sw = GetScreenWidth();
    int ei_sh = GetScreenHeight();
    float ei_margin = 40.0f;
    float ei_scale = powf(ei_sh / 720.0f, 0.7f);
    if (ei_scale < 1.0f) ei_scale = 1.0f;
    Vector3 cam_fwd = Vector3Normalize(Vector3Subtract(
        camera.target, camera.position));

    for (int i = 0; i < vehicle_count; i++) {
        if (i == selected || !vehicles[i].active) continue;

        Vector3 to_drone = Vector3Subtract(vehicles[i].position,
                                            camera.position);
        float dot = to_drone.x * cam_fwd.x + to_drone.y * cam_fwd.y
                    + to_drone.z * cam_fwd.z;

        Vector2 sp = GetWorldToScreen(vehicles[i].position, camera);

        if (sp.x >= ei_margin && sp.x <= ei_sw - ei_margin &&
            sp.y >= ei_margin && sp.y <= ei_sh - ei_margin) continue;

        float ei_cx = ei_sw / 2.0f;
        float ei_cy = ei_sh / 2.0f;
        float ei_dx = sp.x - ei_cx;
        float ei_dy = sp.y - ei_cy;

        if (dot < 0.5f) {
            Vector3 cam_right = Vector3Normalize(
                Vector3CrossProduct(cam_fwd, (Vector3){0, 1, 0}));
            Vector3 cam_up_approx = Vector3CrossProduct(cam_right, cam_fwd);
            ei_dx = Vector3DotProduct(to_drone, cam_right);
            ei_dy = -Vector3DotProduct(to_drone, cam_up_approx);
            float len = sqrtf(ei_dx * ei_dx + ei_dy * ei_dy);
            if (len > 0.01f) { ei_dx /= len; ei_dy /= len; }
            ei_dx *= ei_sw;
            ei_dy *= ei_sh;
        }

        float sx = (ei_dx != 0)
            ? ((ei_dx > 0 ? ei_sw - ei_margin : ei_margin) - ei_cx) / ei_dx
            : 1e9f;
        float sy = (ei_dy != 0)
            ? ((ei_dy > 0 ? ei_sh - ei_margin : ei_margin) - ei_cy) / ei_dy
            : 1e9f;
        float se = fminf(fabsf(sx), fabsf(sy));
        float ex = ei_cx + ei_dx * se;
        float ey = ei_cy + ei_dy * se;
        if (ex < ei_margin) ex = ei_margin;
        if (ex > ei_sw - ei_margin) ex = ei_sw - ei_margin;
        if (ey < ei_margin) ey = ei_margin;
        if (ey > ei_sh - ei_margin) ey = ei_sh - ei_margin;

        Color col = vehicles[i].color;
        col.a = 220;
        float angle = atan2f(ei_dy, ei_dx);
        float sz = 14.0f * ei_scale;

        // Chevron
        float chev_len = sz * 1.2f;
        float chev_spread = 0.5f;
        Vector2 tip = { ex + cosf(angle) * chev_len,
                        ey + sinf(angle) * chev_len };
        Vector2 cl = { ex + cosf(angle + chev_spread) * sz * 0.6f,
                       ey + sinf(angle + chev_spread) * sz * 0.6f };
        Vector2 cr = { ex + cosf(angle - chev_spread) * sz * 0.6f,
                       ey + sinf(angle - chev_spread) * sz * 0.6f };
        DrawLineEx(tip, cl, 2.5f * ei_scale, col);
        DrawLineEx(tip, cr, 2.5f * ei_scale, col);

        // Drone number
        char num[4];
        snprintf(num, sizeof(num), "%d", i + 1);
        float lfs = 18.0f * ei_scale;
        Vector2 tw = MeasureTextEx(font, num, lfs, 0.5f);
        float lx = ex - cosf(angle) * (sz * 0.3f) - tw.x / 2;
        float ly = ey - sinf(angle) * (sz * 0.3f) - tw.y / 2;
        DrawTextEx(font, num, (Vector2){ lx, ly }, lfs, 0.5f, col);
    }
}

static void draw_density_heatmap(const vehicle_t *vehicles, int vehicle_count,
                                 const theme_t *theme) {
    enum { HEAT_COLS = 14, HEAT_ROWS = 12 };
    int density[HEAT_COLS * HEAT_ROWS] = {0};
    float min_x = INFINITY, max_x = -INFINITY;
    float min_z = INFINITY, max_z = -INFINITY;
    int active = 0;

    for (int i = 0; i < vehicle_count; i++) {
        if (!vehicles[i].active) continue;
        float x = vehicles[i].position.x;
        float z = vehicles[i].position.z;
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (z < min_z) min_z = z;
        if (z > max_z) max_z = z;
        active++;
    }
    if (!active) return;
    if (max_x - min_x < 1.0f) { min_x -= 0.5f; max_x += 0.5f; }
    if (max_z - min_z < 1.0f) { min_z -= 0.5f; max_z += 0.5f; }

    for (int i = 0; i < vehicle_count; i++) {
        if (!vehicles[i].active) continue;
        int x = (int)((vehicles[i].position.x - min_x) / (max_x - min_x) * HEAT_COLS);
        int z = (int)((vehicles[i].position.z - min_z) / (max_z - min_z) * HEAT_ROWS);
        if (x == HEAT_COLS) x--;
        if (z == HEAT_ROWS) z--;
        density[z * HEAT_COLS + x]++;
    }

    const float cell_x = (max_x - min_x) / HEAT_COLS;
    const float cell_z = (max_z - min_z) / HEAT_ROWS;
    for (int z = 0; z < HEAT_ROWS; z++) {
        for (int x = 0; x < HEAT_COLS; x++) {
            int count = density[z * HEAT_COLS + x];
            if (!count) continue;
            Color color = theme->hud_accent;
            int alpha = 20 + count * 18;
            color.a = (unsigned char)(alpha > 110 ? 110 : alpha);
            DrawCube((Vector3){min_x + (x + 0.5f) * cell_x, 0.015f,
                               min_z + (z + 0.5f) * cell_z},
                     cell_x * 0.94f, 0.02f, cell_z * 0.94f, color);
        }
    }
}

int main(int argc, char *argv[]) {
    uint16_t base_port = 19410;
    int vehicle_count = 1;
    int model_idx = MODEL_QUADROTOR;
    int win_w = 1280;
    int win_h = 720;
    bool debug = false;
    // PX4 SIH default spawn position
    double origin_lat = 47.397742;
    double origin_lon = 8.545594;
    double origin_alt = 489.4;
    bool origin_specified = false;
    char **replay_paths = calloc(VEHICLE_SANITY_LIMIT, sizeof(*replay_paths));
    if (!replay_paths) return 1;
    int num_replay_files = 0;
    bool ghost_mode = false;

    // Shared fleet map. Every source feeds the same one; the source kind only
    // decides who does the decoding.
    typedef enum { SRC_MAVLINK, SRC_ULOG, SRC_TLOG, SRC_BIN } src_kind_t;
    src_kind_t replay_kind = SRC_ULOG;
    const char *record_path = NULL;
    const char *run_manifest = NULL;
    bool  map_enabled = true;
    double map_cap_mib = 256.0;
    double map_res_m = 0.25;
    fleet_origin_policy_t map_origin_policy = FLEET_ORIGIN_FIRST_SEEN;

    capture_t capture;
    capture_defaults(&capture);

    // Framing for an unattended run. Nobody is there to press Alt+2 or drag the
    // camera onto the map, so the view has to be settable from the command line
    // for a recording to show anything worth recording.
    ortho_mode_t start_view = ORTHO_NONE;
    double view_span_m = 0.0;      // 0 = fit to the map
    bool   follow_map = false;

    for (int i = 1; i < argc; i++) {
        const int taken = capture_parse_arg(&capture, argc, argv, i);
        if (taken > 0) { i += taken - 1; continue; }
        if (strcmp(argv[i], "-udp") == 0 && i + 1 < argc) {
            base_port = (uint16_t)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            char *end = NULL;
            long requested = strtol(argv[++i], &end, 10);
            if (!end || *end != '\0' || requested < 1 || requested > VEHICLE_SANITY_LIMIT) {
                fprintf(stderr, "Invalid vehicle count '%s': -n must be between 1 and %d\n",
                        argv[i], VEHICLE_SANITY_LIMIT);
                free(replay_paths);
                return 1;
            }
            vehicle_count = (int)requested;
        } else if (strcmp(argv[i], "-origin") == 0 && i + 3 < argc) {
            origin_lat = atof(argv[++i]);
            origin_lon = atof(argv[++i]);
            origin_alt = atof(argv[++i]);
            origin_specified = true;
        } else if (strcmp(argv[i], "-mc") == 0) {
            model_idx = MODEL_QUADROTOR;
        } else if (strcmp(argv[i], "-fw") == 0) {
            model_idx = MODEL_FIXEDWING;
        } else if (strcmp(argv[i], "-ts") == 0) {
            model_idx = MODEL_TAILSITTER;
        } else if (strcmp(argv[i], "-d") == 0) {
            debug = true;
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            win_w = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 && i + 1 < argc) {
            win_h = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--replay") == 0) {
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                if (num_replay_files >= VEHICLE_SANITY_LIMIT) {
                    fprintf(stderr, "Too many replay files (max %d)\n", VEHICLE_SANITY_LIMIT);
                    return 1;
                }
                replay_paths[num_replay_files++] = argv[++i];
            }
        } else if (strcmp(argv[i], "--ghost") == 0) {
            ghost_mode = true;
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                if (num_replay_files >= VEHICLE_SANITY_LIMIT) {
                    fprintf(stderr, "Too many replay files (max %d)\n", VEHICLE_SANITY_LIMIT);
                    return 1;
                }
                replay_paths[num_replay_files++] = argv[++i];
            }
        } else if (strcmp(argv[i], "--tlog") == 0 || strcmp(argv[i], "--bin") == 0) {
            replay_kind = (strcmp(argv[i], "--tlog") == 0) ? SRC_TLOG : SRC_BIN;
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                if (num_replay_files >= VEHICLE_SANITY_LIMIT) {
                    fprintf(stderr, "Too many replay files (max %d)\n", VEHICLE_SANITY_LIMIT);
                    free(replay_paths);
                    return 1;
                }
                replay_paths[num_replay_files++] = argv[++i];
            }
        } else if (strcmp(argv[i], "--run") == 0 && i + 1 < argc) {
            run_manifest = argv[++i];
        } else if (strcmp(argv[i], "--record") == 0 && i + 1 < argc) {
            record_path = argv[++i];
        } else if (strcmp(argv[i], "--no-map") == 0) {
            map_enabled = false;
        } else if (strcmp(argv[i], "--map-cap") == 0 && i + 1 < argc) {
            map_cap_mib = atof(argv[++i]);
        } else if (strcmp(argv[i], "--map-res") == 0 && i + 1 < argc) {
            map_res_m = atof(argv[++i]);
        } else if (strcmp(argv[i], "--map-origin") == 0 && i + 1 < argc) {
            map_origin_policy = (strcmp(argv[++i], "centroid") == 0)
                ? FLEET_ORIGIN_CENTROID : FLEET_ORIGIN_FIRST_SEEN;
        } else if (strcmp(argv[i], "--view") == 0 && i + 1 < argc) {
            const char *v = argv[++i];
            if      (strcmp(v, "top") == 0)    start_view = ORTHO_TOP;
            else if (strcmp(v, "bottom") == 0) start_view = ORTHO_BOTTOM;
            else if (strcmp(v, "front") == 0)  start_view = ORTHO_FRONT;
            else if (strcmp(v, "back") == 0)   start_view = ORTHO_BACK;
            else if (strcmp(v, "left") == 0)   start_view = ORTHO_LEFT;
            else if (strcmp(v, "right") == 0)  start_view = ORTHO_RIGHT;
            else if (strcmp(v, "chase") == 0)  start_view = ORTHO_NONE;
            else { fprintf(stderr, "unknown --view %s\n", v); return 1; }
            follow_map = true;
        } else if (strcmp(argv[i], "--view-span") == 0 && i + 1 < argc) {
            view_span_m = atof(argv[++i]);
        } else if (strcmp(argv[i], "--follow-map") == 0) {
            follow_map = true;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            capture_usage();
            return 0;
        }
    }

    // A skynet run record is a manifest, not a log: it names the sources and we
    // open them with the parsers that already exist.
    skynet_manifest_t manifest;
    bool have_manifest = false;
    if (run_manifest) {
        char err[256] = {0};
        if (skynet_manifest_load(&manifest, run_manifest, err, sizeof(err)) != 0) {
            fprintf(stderr, "Failed to open run record %s: %s\n", run_manifest, err);
            free(replay_paths);
            return 1;
        }
        have_manifest = true;
        num_replay_files = 0;
        for (int e = 0; e < manifest.entry_count && num_replay_files < VEHICLE_SANITY_LIMIT; e++)
            replay_paths[num_replay_files++] = manifest.entries[e].path;
        printf("Run %s: %d log(s)\n",
               manifest.run_id[0] ? manifest.run_id : run_manifest, num_replay_files);
    }

    if ((unsigned int)base_port + (unsigned int)vehicle_count > 65535U) {
        fprintf(stderr, "Invalid UDP port range: base port %u with %d vehicles exceeds 65535\n",
                base_port, vehicle_count);
        free(replay_paths);
        return 1;
    }

    asset_path_init();

    // Init Raylib
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(win_w, win_h, "Hawkeye");
    SetTargetFPS(60);

    // Init data sources
    bool is_replay = (num_replay_files > 0);
    if (is_replay) vehicle_count = num_replay_files;
    data_source_t *sources = calloc((size_t)vehicle_count, sizeof(*sources));
    if (!sources) {
        fprintf(stderr, "Failed to allocate data sources for %d vehicles\n", vehicle_count);
        free(replay_paths);
        CloseWindow();
        return 1;
    }

    // One shared map for the whole fleet, created before the sources so each
    // source can be attached to it as it opens.
    map_session_t map_session;
    bool map_ready = false;
    if (map_enabled) {
        map_session_config_t mcfg;
        map_session_config_defaults(&mcfg);
        mcfg.map_byte_cap = (size_t)(map_cap_mib * 1024.0 * 1024.0);
        mcfg.max_vehicles = vehicle_count;
        mcfg.origin_policy = map_origin_policy;
        // Leaf size sets the depth under the fixed 4096 m root.
        if (map_res_m > 0.0) {
            int depth = (int)ceil(log2(4096.0 / map_res_m));
            if (depth < 6) depth = 6;
            if (depth > 16) depth = 16;
            mcfg.max_depth = depth;
            mcfg.coarse_depth = depth - 3;   // ~8x coarser for far-field carving
        }
        if (map_session_init(&map_session, &mcfg) == 0) {
            map_ready = true;
            if (origin_specified)
                fleet_frame_set_explicit(&map_session.frame, origin_lat, origin_lon, origin_alt);
        } else {
            fprintf(stderr, "Failed to create the shared map; continuing without it\n");
        }
    }
    struct map_session *map_ptr = map_ready ? (struct map_session *)&map_session : NULL;

    if (is_replay) {
        for (int i = 0; i < num_replay_files; i++) {
            int rc = -1;
            src_kind_t kind = replay_kind;
            if (have_manifest && i < manifest.entry_count) {
                switch (manifest.entries[i].kind) {
                    case SKYNET_LOG_TLOG:      kind = SRC_TLOG; break;
                    case SKYNET_LOG_DATAFLASH: kind = SRC_BIN;  break;
                    default:                   kind = SRC_ULOG; break;
                }
            }
            switch (kind) {
                case SRC_TLOG:
                    rc = data_source_tlog_create(&sources[i], replay_paths[i],
                                                 map_ptr, i, (uint8_t)i);
                    break;
                case SRC_BIN:
                    rc = data_source_bin_create(&sources[i], replay_paths[i], map_ptr, i);
                    break;
                default:
                    rc = data_source_ulog_create(&sources[i], replay_paths[i]);
                    if (rc == 0) data_source_attach_map(&sources[i], map_ptr, i);
                    break;
            }
            if (rc != 0) {
                fprintf(stderr, "Failed to open %s\n", replay_paths[i]);
                free(replay_paths); free(sources);
                CloseWindow();
                return 1;
            }
            if (map_ready) map_session_bind_slot(&map_session, i, (uint8_t)(i + 1));
            if (have_manifest && i < manifest.entry_count && manifest.entries[i].has_offset)
                data_source_set_time_offset(&sources[i], manifest.entries[i].time_offset_s);
        }
    } else {
        for (int i = 0; i < vehicle_count; i++) {
            if (data_source_mavlink_create(&sources[i], base_port + i, (uint8_t)i, debug) != 0) {
                fprintf(stderr, "Failed to init MAVLink receiver on port %u\n", base_port + i);
                free(replay_paths); free(sources);
                CloseWindow();
                return 1;
            }
            data_source_attach_map(&sources[i], map_ptr, i);
            if (map_ready) map_session_bind_slot(&map_session, i, (uint8_t)(i + 1));

            // The tlog recorder is the only thing that preserves latency, loss
            // and ordering as the viewer actually saw them.
            if (record_path) {
                char path[1024];
                if (vehicle_count > 1) snprintf(path, sizeof(path), "%s.%d", record_path, i);
                else snprintf(path, sizeof(path), "%s", record_path);
                if (data_source_mavlink_record(&sources[i], path) != 0)
                    fprintf(stderr, "Failed to open tlog for writing: %s\n", path);
                else
                    printf("Recording vehicle %d to %s\n", i, path);
            }
        }
    }

    map_render_t map_render;
    bool map_render_ready = map_ready && (map_render_init(&map_render) == 0);
    map_hud_opts_t map_hud_opts;
    map_hud_defaults(&map_hud_opts);
    quality_overlay_opts_t quality_opts;
    quality_overlay_defaults(&quality_opts);
    bool show_map_panel = map_ready;

    // Init vehicles
    // Init scene first (provides lighting shader for vehicles)
    scene_t scene;
    scene_init(&scene);

    vehicle_t *vehicles = calloc((size_t)vehicle_count, sizeof(*vehicles));
    corr_state_t *corr = calloc((size_t)vehicle_count, sizeof(*corr));
    if (!vehicles || !corr) {
        fprintf(stderr, "Failed to allocate state for %d vehicles\n", vehicle_count);
        free(replay_paths); free(sources); free(vehicles); free(corr);
        CloseWindow();
        return 1;
    }
    if (is_replay) {
        // Persistent trail for replay: 36000 points (~10+ min at adaptive rate)
        for (int i = 0; i < num_replay_files; i++) {
            vehicle_init_ex(&vehicles[i], model_idx, scene.lighting_shader, 36000);
            vehicles[i].color = scene.theme->drone_palette[i % THEME_DRONE_PALETTE_SIZE];
        }
    } else {
        for (int i = 0; i < vehicle_count; i++) {
            vehicle_init(&vehicles[i], model_idx, scene.lighting_shader);
            vehicles[i].color = scene.theme->drone_palette[i % THEME_DRONE_PALETTE_SIZE];
        }
    }

    // For multi-vehicle MAVLink or explicit origin: pre-set the NED origin
    if (!is_replay && (vehicle_count > 1 || origin_specified)) {
        double lat0_rad = origin_lat * (M_PI / 180.0);
        double lon0_rad = origin_lon * (M_PI / 180.0);
        for (int i = 0; i < vehicle_count; i++) {
            vehicles[i].lat0 = lat0_rad;
            vehicles[i].lon0 = lon0_rad;
            vehicles[i].alt0 = origin_alt;
            vehicles[i].origin_set = true;
        }
        printf("NED origin: lat=%.6f lon=%.6f alt=%.1f\n", origin_lat, origin_lon, origin_alt);
    } else if (origin_specified) {
        double lat0_rad = origin_lat * (M_PI / 180.0);
        double lon0_rad = origin_lon * (M_PI / 180.0);
        for (int i = 0; i < vehicle_count; i++) {
            vehicles[i].lat0 = lat0_rad;
            vehicles[i].lon0 = lon0_rad;
            vehicles[i].alt0 = origin_alt;
            vehicles[i].origin_set = true;
        }
    }

    // ── Takeoff alignment state (toggled by A key) ──
    bool takeoff_aligned = false;
    if (is_replay && num_replay_files > 1) {
        // Set multi-file CONF for each source (always available)
        for (int i = 0; i < num_replay_files; i++) {
            // takeoff_conf already populated by data_source_ulog_create
            sources[i].playback.time_offset_s = 0.0f;
        }
    }

    // ── Conflict detection + resolution (multi-file replay, not --ghost) ──
    bool conflict_detected = false;
    bool conflict_far = false;
    bool ghost_mode_grid = false;

    if (is_replay && num_replay_files > 1 && !ghost_mode) {
        conflict_result_t cr = replay_detect_conflict(sources, num_replay_files);
        conflict_detected = cr.conflict_detected;
        conflict_far = cr.conflict_far;

        if (conflict_detected) {
            // Init HUD early for fonts
            hud_t prompt_hud;
            hud_init(&prompt_hud);

            const char *grid_label = conflict_far ? "Narrow grid offset" : "Grid offset";
            char subtitle[64];
            if (conflict_far)
                snprintf(subtitle, sizeof(subtitle), "  -  %d drones too far apart", num_replay_files);
            else
                snprintf(subtitle, sizeof(subtitle), "  -  %d drones overlap", num_replay_files);
            const char *labels[] = {"Cancel & reupload", "Ghost mode", grid_label};

            int choice = draw_prompt_dialog("POSITION CONFLICT", subtitle,
                                            labels, 3, scene.theme,
                                            prompt_hud.font_label, prompt_hud.font_value,
                                            &scene);

            hud_cleanup(&prompt_hud);

            if (choice <= 1) {
                for (int i = 0; i < num_replay_files; i++)
                    data_source_close(&sources[i]);
                vehicle_cleanup(&vehicles[0]);
                scene_cleanup(&scene);
                CloseWindow();
                return 0;
            } else if (choice == 2) {
                ghost_mode = true;
            } else if (choice == 3) {
                ghost_mode_grid = true;
            }
        }
    }

    // Compute shared NED origin for multi-drone replay.
    // ref_lat_rad/ref_lon_rad/min_alt persist for runtime mode switching (P key).
    double ref_lat_rad = 0.0, ref_lon_rad = 0.0, min_alt = 0.0;
    int ref_idx = -1;
    if (is_replay && num_replay_files > 1) {
        for (int i = 0; i < num_replay_files; i++) {
            if (sources[i].home.valid) {
                ref_idx = i;
                ref_lat_rad = sources[i].home.lat / 1e7 * (M_PI / 180.0);
                ref_lon_rad = sources[i].home.lon / 1e7 * (M_PI / 180.0);
                break;
            }
        }

        min_alt = 1e9;
        for (int i = 0; i < num_replay_files; i++) {
            if (sources[i].home.valid) {
                double a = sources[i].home.alt * 1e-3;
                if (a < min_alt) min_alt = a;
            }
        }
        if (min_alt > 1e8) min_alt = 0.0;

        if (ghost_mode || ghost_mode_grid) {
            // Ghost/grid: each drone uses its own home as origin (collapse to center).
            // Grid mode additionally offsets each drone along X.
            // Don't set origin_set — vehicle_update will set each drone's own origin.
            if (ghost_mode_grid) {
                for (int i = 1; i < num_replay_files; i++)
                    vehicles[i].grid_offset.x = i * 5.0f;
            }
        } else {
            // Normal replay: shared origin so drones render at real relative positions
            for (int i = 0; i < num_replay_files; i++) {
                if (sources[i].home.valid) {
                    vehicles[i].lat0 = ref_lat_rad;
                    vehicles[i].lon0 = ref_lon_rad;
                    vehicles[i].alt0 = min_alt;
                    vehicles[i].origin_set = true;
                }
            }
        }

        if (ref_idx >= 0)
            printf("Multi-drone origin: lat=%.6f lon=%.6f alt=%.1f (min datum)\n",
                   ref_lat_rad * (180.0 / M_PI), ref_lon_rad * (180.0 / M_PI), min_alt);
    }

    // Compute position tier per vehicle (for debug panel)
    int *vehicle_tier = calloc((size_t)vehicle_count, sizeof(*vehicle_tier));
    if (is_replay) {
        for (int i = 0; i < num_replay_files; i++) {
            if (sources[i].playback.home_from_topic) vehicle_tier[i] = 1;
            else if (sources[i].home.valid) vehicle_tier[i] = 2;
            else vehicle_tier[i] = 3;
        }
    }

    // Check for Tier 3 drones (no valid home = estimated position)
    bool has_tier3 = false;
    for (int i = 0; i < num_replay_files; i++) {
        if (!sources[i].home.valid) { has_tier3 = true; break; }
    }

    // Apply ghost mode: translucent non-primary drones
    if (ghost_mode && num_replay_files > 1) {
        vehicle_set_ghost_alpha(&vehicles[0], 1.0f);
        for (int i = 1; i < num_replay_files; i++)
            vehicle_set_ghost_alpha(&vehicles[i], 0.35f);
    }

    hud_t hud;
    hud_init(&hud);
    hud.is_replay = is_replay;

    debug_panel_t dbg_panel;
    debug_panel_init(&dbg_panel);

    ortho_panel_t ortho;
    ortho_panel_init(&ortho);

    int selected = 0;
    int prev_selected = 0;
    bool *was_connected = calloc((size_t)vehicle_count, sizeof(*was_connected));
    Vector3 *last_pos = calloc((size_t)vehicle_count, sizeof(*last_pos));
    bool show_hud = true;
    float saved_chase_distance = 0.0f;
    float tactical_chase_target = 1.6f;

    // Click-vs-drag state for ray-picking a vehicle in the 3D view
    Vector2 click_origin = {0};
    bool click_in_view = false;

    // Key chord state for two-digit drone selection (10-16)
    int chord_value = -1;       // accumulated 1-based vehicle number
    double chord_time = 0.0;    // when first digit was pressed
    bool chord_shift = false;   // whether shift was held on first digit
    int trail_mode = (num_replay_files > 1) ? 3 : 1;  // multi-drone defaults to ID trails
    bool show_ground_track = false;  // ground projection off by default
    bool classic_colors = false;     // K key: toggle classic (red/blue) vs modern (yellow/purple)
    bool show_edge_indicators = true; // Ctrl+L: screen edge drone indicators
    int corr_mode = 0;               // Shift+T: 0=off, 1=ribbon, 2=line
    bool show_corr_labels = true;    // Ctrl+L: distance labels in ortho correlation
    bool show_axes = false;          // Z: axis orientation gizmo
    bool *insufficient_data = calloc((size_t)vehicle_count, sizeof(*insufficient_data));
    float *prev_playback_pos = calloc((size_t)vehicle_count, sizeof(*prev_playback_pos));
    int insufficient_check_frames = 0;
    bool insufficient_toasted = false;

    // Per-drone markers, system markers, and pre-computed trails
    user_markers_t *markers = calloc((size_t)vehicle_count, sizeof(*markers));
    sys_markers_t *sys_markers = calloc((size_t)vehicle_count, sizeof(*sys_markers));
    precomp_trail_t *precomp = calloc((size_t)vehicle_count, sizeof(*precomp));
    hud_marker_data_t *all_user_md = calloc((size_t)vehicle_count, sizeof(*all_user_md));
    hud_marker_data_t *all_sys_md = calloc((size_t)vehicle_count, sizeof(*all_sys_md));
    if (!vehicle_tier || !was_connected || !last_pos || !insufficient_data ||
        !prev_playback_pos || !markers || !sys_markers || !precomp ||
        !all_user_md || !all_sys_md) {
        fprintf(stderr, "Failed to allocate per-vehicle UI state for %d vehicles\n",
                vehicle_count);
        free(replay_paths);
        free(sources); free(vehicles); free(corr); free(vehicle_tier);
        free(was_connected); free(last_pos); free(insufficient_data);
        free(prev_playback_pos); free(markers); free(sys_markers); free(precomp);
        free(all_user_md); free(all_sys_md);
        CloseWindow();
        return 1;
    }
    for (int i = 0; i < vehicle_count; i++) {
        markers[i].current = -1;
        markers[i].last_drop_idx = -1;
        sys_markers[i].current = -1;
        precomp_trail_init(&precomp[i]);
        if (is_replay)
            replay_init_sys_markers(&sys_markers[i], &sources[i]);
    }

    // Marker label input state
    bool show_marker_labels = true;
    marker_input_t marker_input = {0};
    marker_input.target = -1;

    if (start_view != ORTHO_NONE) {
        scene.ortho_mode = start_view;
        scene.ortho_span = (view_span_m > 0.0) ? (float)view_span_m : 60.0f;
    }
    if (follow_map && !map_ready)
        fprintf(stderr, "hawkeye: --follow-map has nothing to follow with --no-map\n");

    if (!capture_begin(&capture, "hawkeye")) return 1;

    // Main loop
    while (!WindowShouldClose()) {
        // Guard: if vehicle_count is somehow 0, exit the loop to avoid
        // out-of-bounds access on sources[selected] / vehicles[selected].
        if (vehicle_count <= 0) break;

        // Poll all data sources and update vehicles
        for (int i = 0; i < vehicle_count; i++) {
            data_source_poll(&sources[i], GetFrameTime());

            // Feed STATUSTEXT messages into HUD ticker
            if (sources[i].playback.statustext)
                hud_feed_statustext(&hud, sources[i].playback.statustext, i);

            // Check if playback crossed any marker times (annunciator triggers)
            if (is_replay && !sources[i].playback.paused) {
                float cur_pos = sources[i].playback.position_s;
                float prev_pos = prev_playback_pos[i];
                // User markers
                for (int m = 0; m < markers[i].count; m++) {
                    if (markers[i].times[m] > prev_pos && markers[i].times[m] <= cur_pos) {
                        annunc_trigger_tab_fade(&hud.annunciators, i);
                        annunc_trigger_radar_wave(&hud.annunciators, i);
                        if (i != selected) {
                            for (int p = 0; p < hud.pinned_count; p++)
                                if (hud.pinned[p] == i)
                                    annunc_trigger_ring_bounce(&hud.annunciators, i);
                        }
                        break;
                    }
                }
                // System markers
                for (int m = 0; m < sys_markers[i].count; m++) {
                    if (sys_markers[i].times[m] > prev_pos && sys_markers[i].times[m] <= cur_pos) {
                        annunc_trigger_tab_fade(&hud.annunciators, i);
                        annunc_trigger_radar_wave(&hud.annunciators, i);
                        if (i != selected) {
                            for (int p = 0; p < hud.pinned_count; p++)
                                if (hud.pinned[p] == i)
                                    annunc_trigger_ring_bounce(&hud.annunciators, i);
                        }
                        break;
                    }
                }
                prev_playback_pos[i] = cur_pos;
            }

            // Reset trail and origin on reconnect
            if (sources[i].connected && !was_connected[i]) {
                vehicle_reset_trail(&vehicles[i]);
                if (sources[i].mav_type != 0)
                    vehicle_set_type(&vehicles[i], sources[i].mav_type);
                if (!origin_specified && vehicle_count == 1) {
                    vehicles[i].origin_set = false;
                    vehicles[i].origin_wait_count = 0;
                }
            }
            was_connected[i] = sources[i].connected;

            vehicles[i].current_time = sources[i].playback.position_s;
            vehicle_update(&vehicles[i], &sources[i].state, &sources[i].home);
            vehicles[i].sysid = sources[i].sysid;

            // Placeholder: show model at origin while waiting for position data.
            if (sources[i].connected && !vehicles[i].active) {
                vehicles[i].active = true;
                vehicles[i].position = vehicles[i].grid_offset;
            }

            // Detect position jump (new SITL connecting before disconnect timeout)
            if (vehicles[i].active && vehicles[i].trail_count > 0) {
                Vector3 delta = Vector3Subtract(vehicles[i].position, last_pos[i]);
                if (Vector3Length(delta) > 50.0f) {
                    vehicle_reset_trail(&vehicles[i]);
                }
            }
            last_pos[i] = vehicles[i].position;
        }

        // Advance the shared map. Live pins the playhead to the head of the
        // data; scrubbing unpins it and the map is rebuilt to match. One code
        // path, and the live view gets rewind for free.
        if (map_ready) {
            map_session_set_focus(&map_session, selected);
            if (is_replay && vehicle_count > 0) {
                // Replay drives the playhead from the transport position so the
                // map and the trajectory never disagree about "now". Each
                // source clamps its own position at its own end of log, so the
                // shared playhead has to come from the furthest one still
                // running -- otherwise loading a 60 s log beside a 600 s one
                // freezes the map nine tenths of the way short while the other
                // vehicle keeps flying.
                const int64_t t0 = timeline_start_ns(&map_session.timeline);
                double furthest = 0.0;
                for (int i = 0; i < vehicle_count; i++) {
                    if (sources[i].playback.position_s > furthest)
                        furthest = sources[i].playback.position_s;
                }
                const int64_t want = t0 + (int64_t)(furthest * 1e9);
                map_session.timeline.playhead.speed = sources[0].playback.speed;
                map_session.timeline.playhead.paused = sources[0].playback.paused;
                map_session_tick_at(&map_session, want, GetFrameTime());
            } else {
                map_session_tick(&map_session, GetFrameTime());
            }
        }

        // Lazy-resolve system marker positions once origin is established
        for (int i = 0; i < vehicle_count; i++) {
            if (is_replay && !sys_markers[i].resolved && sys_markers[i].count > 0
                && vehicles[i].origin_set) {
                replay_resolve_and_build_trail(&sys_markers[i], &precomp[i],
                                               &sources[i], &vehicles[i]);
            }
        }

        // Detect drones with insufficient position data (~2s after start)
        if (is_replay && !insufficient_toasted) {
            insufficient_check_frames++;
            if (insufficient_check_frames > 120) {
                for (int i = 0; i < vehicle_count; i++) {
                    if (sources[i].connected && !sources[i].state.valid) {
                        insufficient_data[i] = true;
                        char msg[64];
                        snprintf(msg, sizeof(msg), "DRONE %d: INSUFFICIENT DATA", i + 1);
                        hud_toast_color(&hud, msg, 4.0f, vehicles[i].color);
                    }
                }
                insufficient_toasted = true;
            }
        }

        // Grid offsets for ghost/narrow-grid are now computed at init time
        // from home positions (see shared origin block above).

        // Positional correlation vs selected drone (only while playing)
        if (is_replay && num_replay_files > 1) {
            // Reset accumulators when selection changes
            if (selected != prev_selected) {
                memset(corr, 0, (size_t)vehicle_count * sizeof(*corr));
                // Reset the whole fleet, not just the page we are about to
                // accumulate: off-page drones would otherwise keep displaying
                // correlations computed against the previous reference.
                for (int i = 0; i < num_replay_files; i++) {
                    sources[i].playback.correlation = NAN;
                    sources[i].playback.rmse = NAN;
                }
                prev_selected = selected;
            }

            // Only accumulate while playback is active (not paused, not ended)
            bool playing = sources[selected].connected && !sources[selected].playback.paused;
            if (playing && vehicles[selected].origin_set) {
                const vehicle_t *ref = &vehicles[selected];
                double rx[CORR_CHANNELS] = {
                    ref->position.z, ref->position.x, ref->position.y
                };
                int page_first = (selected / FLEET_PAGE_SIZE) * FLEET_PAGE_SIZE;
                int page_last = page_first + FLEET_PAGE_SIZE;
                if (page_last > num_replay_files) page_last = num_replay_files;
                for (int i = page_first; i < page_last; i++) {
                    if (i == selected || !vehicles[i].origin_set) continue;
                    const vehicle_t *v = &vehicles[i];
                    double vx[CORR_CHANNELS] = {
                        v->position.z, v->position.x, v->position.y
                    };
                    for (int c = 0; c < CORR_CHANNELS; c++) {
                        corr[i].ch[c].sum_x  += rx[c];
                        corr[i].ch[c].sum_y  += vx[c];
                        corr[i].ch[c].sum_xy += rx[c] * vx[c];
                        corr[i].ch[c].sum_x2 += rx[c] * rx[c];
                        corr[i].ch[c].sum_y2 += vx[c] * vx[c];
                    }
                    // Euclidean distance squared for RMSE (subtract grid offsets)
                    double dx = (v->position.x - v->grid_offset.x) - (ref->position.x - ref->grid_offset.x);
                    double dy = (v->position.y - v->grid_offset.y) - (ref->position.y - ref->grid_offset.y);
                    double dz = (v->position.z - v->grid_offset.z) - (ref->position.z - ref->grid_offset.z);
                    corr[i].sum_sq_dist += dx*dx + dy*dy + dz*dz;
                    corr[i].n++;
                    sources[i].playback.correlation = corr_compute(&corr[i]);
                    sources[i].playback.rmse = (corr[i].n >= CORR_MIN_SAMPLES)
                        ? (float)sqrt(corr[i].sum_sq_dist / corr[i].n) : NAN;
                }
                sources[selected].playback.correlation = 1.0f;
                sources[selected].playback.rmse = 0.0f;
            }
        }

        // Check if any source is connected (for HUD)
        bool any_connected = false;
        for (int i = 0; i < vehicle_count; i++) {
            if (sources[i].connected) { any_connected = true; break; }
        }

        // Update HUD sim time from selected vehicle
        hud_update(&hud, sources[selected].state.time_usec,
                   sources[selected].connected, GetFrameTime());

        // Handle file drops (.mvt theme files)
        if (IsFileDropped()) {
            FilePathList dropped = LoadDroppedFiles();
            for (unsigned int i = 0; i < dropped.count; i++) {
                if (theme_registry_add(&scene.theme_reg, dropped.paths[i])) {
                    int last = scene.theme_reg.user_count - 1;
                    char msg[80];
                    snprintf(msg, sizeof(msg), "Theme: %s", scene.theme_reg.name_bufs[last]);
                    hud_toast(&hud, msg, 3.0f);

                    // Copy to themes/ so it persists
                    const char *src = dropped.paths[i];
                    const char *fname = src;
                    for (const char *p = src; *p; p++) {
                        if (*p == '/' || *p == '\\') fname = p + 1;
                    }
                    char dest[512];
                    snprintf(dest, sizeof(dest), "./themes/%s", fname);
                    // Ensure themes/ exists, then copy if not already there
                    #ifdef _WIN32
                    _mkdir("./themes");
                    #else
                    mkdir("./themes", 0755);
                    #endif
                    if (strcmp(src, dest) != 0) {
                        FILE *fin = fopen(src, "rb");
                        FILE *fout = fin ? fopen(dest, "wb") : NULL;
                        if (fin && fout) {
                            char buf[4096];
                            size_t n;
                            while ((n = fread(buf, 1, sizeof(buf), fin)) > 0)
                                fwrite(buf, 1, n, fout);
                            printf("Theme saved to %s\n", dest);
                        }
                        if (fin) fclose(fin);
                        if (fout) fclose(fout);
                    }
                }
            }
            UnloadDroppedFiles(dropped);
        }

        // Handle input (blocked during marker label entry)
        if (!marker_input.active) {
        // The timeline widget gets the pointer first. It is drawn later, inside
        // BeginDrawing, but its hit test has to run before the camera sees the
        // same mouse button -- otherwise scrubbing the strip also orbits the
        // view.
        scene.ui_pointer_captured = false;
        if (map_ready && map_hud_opts.show_timeline) {
            const int th = (int)map_hud_opts.timeline_height;
            scene.ui_pointer_captured =
                map_hud_timeline_input(&map_session, 12, GetScreenHeight() - th - 12,
                                       GetScreenWidth() - 24, th);
        }
        scene_handle_input(&scene);

        // Map controls. Deliberately few: the map is a view of the same data,
        // not a separate application.
        if (map_ready) {
            const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            if (IsKeyPressed(KEY_J) && map_render_ready)
                map_render_cycle_mode(&map_render, shift ? -1 : 1);
            if (IsKeyPressed(KEY_U) && map_render_ready) {
                if (shift) { map_render.show_free = !map_render.show_free;
                             map_render_invalidate(&map_render); }
                else map_render.visible = !map_render.visible;
            }
            if (IsKeyPressed(KEY_V)) {
                if (shift) quality_opts.show_panel = !quality_opts.show_panel;
                else show_map_panel = !show_map_panel;
            }
            if (IsKeyPressed(KEY_X)) {
                // Per-vehicle solo/mute: which vehicles are allowed to write to
                // the shared map.
                if (shift)
                    map_session_set_solo(&map_session,
                                         map_session.solo_vehicle == selected ? -1 : selected);
                else
                    map_session_set_mute(&map_session, selected,
                                         !map_session.veh[selected].muted);
            }
            if (IsKeyPressed(KEY_HOME)) {
                timeline_pin_live(&map_session.timeline);
                if (map_render_ready) map_render_invalidate(&map_render);
            }
        }

        // P key: switch between Swarm / Ghost / Grid modes during multi-file replay
        // P key: mode switcher for multi-file replay
        // - No conflict: Formation / Ghost / Grid
        // - Conflict (close/no-data): Cancel / Ghost / Grid offset
        // - Conflict (too far): Cancel / Ghost / Narrow grid
        if (IsKeyPressed(KEY_P) && is_replay && num_replay_files > 1) {
            const char *p_title;
            const char *pl[3];
            const char *p_grid_label = conflict_far ? "Narrow grid" : "Grid offset";
            char p_subtitle[64];
            if (conflict_detected) {
                p_title = "POSITION CONFLICT";
                pl[0] = "Cancel"; pl[1] = "Ghost mode"; pl[2] = p_grid_label;
                if (conflict_far)
                    snprintf(p_subtitle, sizeof(p_subtitle), "  -  %d drones too far apart", num_replay_files);
                else
                    snprintf(p_subtitle, sizeof(p_subtitle), "  -  %d drones overlap", num_replay_files);
            } else {
                p_title = "REPLAY MODE";
                pl[0] = "Formation"; pl[1] = "Ghost"; pl[2] = "Grid offset";
                snprintf(p_subtitle, sizeof(p_subtitle), "  -  %d drones", num_replay_files);
            }

            int ch = draw_prompt_dialog(p_title, p_subtitle,
                                        pl, 3, scene.theme,
                                        hud.font_label, hud.font_value,
                                        &scene);
            if (ch == 1) {
                if (conflict_detected) {
                    // Cancel — reset each drone to own origin
                    ghost_mode = false; ghost_mode_grid = false;
                    for (int i = 0; i < num_replay_files; i++) {
                        vehicles[i].grid_offset = (Vector3){0,0,0};
                        vehicle_set_ghost_alpha(&vehicles[i], 1.0f);
                        if (sources[i].home.valid) {
                            vehicles[i].lat0 = sources[i].home.lat * 1e-7 * (M_PI / 180.0);
                            vehicles[i].lon0 = sources[i].home.lon * 1e-7 * (M_PI / 180.0);
                            vehicles[i].alt0 = sources[i].home.alt * 1e-3;
                            vehicles[i].origin_set = true;
                        }
                        vehicles[i].origin_wait_count = 0;
                        vehicle_reset_trail(&vehicles[i]);
                    }
                } else {
                    // Formation mode — shared origin, real relative positions
                    ghost_mode = false; ghost_mode_grid = false;
                    for (int i = 0; i < num_replay_files; i++) {
                        vehicles[i].grid_offset = (Vector3){0,0,0};
                        vehicle_set_ghost_alpha(&vehicles[i], 1.0f);
                        if (sources[i].home.valid) {
                            vehicles[i].lat0 = ref_lat_rad;
                            vehicles[i].lon0 = ref_lon_rad;
                            vehicles[i].alt0 = min_alt;
                            vehicles[i].origin_set = true;
                        }
                        vehicles[i].origin_wait_count = 0;
                        vehicle_reset_trail(&vehicles[i]);
                    }
                }
            } else if (ch == 2) {
                // Ghost mode — each drone uses own home, collapse to center
                ghost_mode = true; ghost_mode_grid = false;
                for (int i = 0; i < num_replay_files; i++) {
                    vehicles[i].grid_offset = (Vector3){0,0,0};
                    if (sources[i].home.valid) {
                        vehicles[i].lat0 = sources[i].home.lat * 1e-7 * (M_PI / 180.0);
                        vehicles[i].lon0 = sources[i].home.lon * 1e-7 * (M_PI / 180.0);
                        vehicles[i].alt0 = sources[i].home.alt * 1e-3;
                        vehicles[i].origin_set = true;
                    }
                    vehicles[i].origin_wait_count = 0;
                    vehicle_reset_trail(&vehicles[i]);
                }
                vehicle_set_ghost_alpha(&vehicles[0], 1.0f);
                for (int i = 1; i < num_replay_files; i++)
                    vehicle_set_ghost_alpha(&vehicles[i], 0.35f);
            } else if (ch == 3) {
                if (conflict_far) {
                    // Narrow grid — each drone keeps own origin, narrow spacing
                    ghost_mode = false; ghost_mode_grid = true;
                    for (int i = 0; i < num_replay_files; i++) {
                        vehicle_set_ghost_alpha(&vehicles[i], 1.0f);
                        if (sources[i].home.valid) {
                            vehicles[i].lat0 = sources[i].home.lat * 1e-7 * (M_PI / 180.0);
                            vehicles[i].lon0 = sources[i].home.lon * 1e-7 * (M_PI / 180.0);
                            vehicles[i].alt0 = sources[i].home.alt * 1e-3;
                            vehicles[i].origin_set = true;
                        }
                        vehicles[i].origin_wait_count = 0;
                        vehicle_reset_trail(&vehicles[i]);
                        vehicles[i].grid_offset = (i > 0) ? (Vector3){ i * 5.0f, 0.0f, 0.0f } : (Vector3){0,0,0};
                    }
                } else {
                    // Grid offset — each drone uses own home + X offset
                    ghost_mode = false; ghost_mode_grid = true;
                    for (int i = 0; i < num_replay_files; i++) {
                        vehicle_set_ghost_alpha(&vehicles[i], 1.0f);
                        if (sources[i].home.valid) {
                            vehicles[i].lat0 = sources[i].home.lat * 1e-7 * (M_PI / 180.0);
                            vehicles[i].lon0 = sources[i].home.lon * 1e-7 * (M_PI / 180.0);
                            vehicles[i].alt0 = sources[i].home.alt * 1e-3;
                            vehicles[i].origin_set = true;
                        }
                        vehicles[i].origin_wait_count = 0;
                        vehicle_reset_trail(&vehicles[i]);
                        vehicles[i].grid_offset = (i > 0) ? (Vector3){ i * 5.0f, 0.0f, 0.0f } : (Vector3){0,0,0};
                    }
                }
            }
            // Reset stats on mode switch
            memset(corr, 0, (size_t)vehicle_count * sizeof(*corr));
            for (int i = 0; i < num_replay_files; i++) {
                sources[i].playback.correlation = NAN;
                sources[i].playback.rmse = NAN;
            }
        }

        // Update vehicle colors when view mode changes
        {
            for (int i = 0; i < vehicle_count; i++)
                vehicles[i].color = scene.theme->drone_palette[i % THEME_DRONE_PALETTE_SIZE];
        }

        // Help overlay toggle (? key = Shift+/)
        if (IsKeyPressed(KEY_SLASH) && (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))) {
            hud.show_help = !hud.show_help;
        }

        // Cycle HUD mode: Console → Tactical → Off
        if (IsKeyPressed(KEY_H)) {
            hud_mode_t prev_mode = hud.mode;
            hud.mode = (hud.mode + 1) % HUD_MODE_COUNT;
            const char *mode_names[] = { "Console HUD", "Tactical HUD", "HUD Off" };
            hud_toast(&hud, mode_names[hud.mode], 2.0f);
            show_hud = (hud.mode != HUD_OFF);

            if (hud.mode == HUD_TACTICAL) {
                saved_chase_distance = scene.chase_distance;
                scene.chase_distance = tactical_chase_target;
            } else if (prev_mode == HUD_TACTICAL) {
                scene.chase_distance = saved_chase_distance;
            }
        }

        // Shift+T: cycle correlation overlay (off → ribbon → line → off)
        if (IsKeyPressed(KEY_T) && (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))
            && is_replay && num_replay_files > 1) {
            corr_mode = (corr_mode + 1) % 3;
            const char *names[] = { "Correlation Off", "Correlation Line", "Correlation Curtain" };
            hud_toast(&hud, names[corr_mode], 2.0f);
        }
        // Cycle trail mode: off → trail → speed ribbon
        else if (IsKeyPressed(KEY_T)) {
            int max_modes = (num_replay_files > 1) ? 4 : 3;
            trail_mode = (trail_mode + 1) % max_modes;
            const char *trail_names[] = { "Trails Off", "Direction Trails", "Speed Ribbons", "ID Trails" };
            hud_toast(&hud, trail_names[trail_mode], 2.0f);
        }

        // Toggle classic/modern arm colors
        if (IsKeyPressed(KEY_K)) {
            classic_colors = !classic_colors;
        }

        // Toggle ground track projection
        if (IsKeyPressed(KEY_G)) {
            show_ground_track = !show_ground_track;
        }

        // Toggle debug panel (Ctrl+D)
        if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_D)) {
            dbg_panel.visible = !dbg_panel.visible;
        }

        // Toggle screen edge indicators and correlation labels (Ctrl+L)
        if ((IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && IsKeyPressed(KEY_L)) {
            show_edge_indicators = !show_edge_indicators;
            show_corr_labels = !show_corr_labels;
        }

        // Toggle axis gizmo (Z)
        if (IsKeyPressed(KEY_Z) && !IsKeyDown(KEY_LEFT_SHIFT) && !IsKeyDown(KEY_RIGHT_SHIFT)
            && !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL)) {
            show_axes = !show_axes;
        }

        // Toggle ortho panel
        if (IsKeyPressed(KEY_O)) {
            ortho.visible = !ortho.visible;
        }

        // Cycle model for selected vehicle
        // Cycle model: M = within group, Shift+M = all models
        if (IsKeyPressed(KEY_M)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                int next = (vehicles[selected].model_idx + 1) % vehicle_model_count;
                vehicle_load_model(&vehicles[selected], next);
            } else {
                vehicle_cycle_model(&vehicles[selected]);
            }
        }

        // Vehicle selection input
        if (vehicle_count > 1) {
            // Pick on release, not press: left-drag orbits the camera (scene.c),
            // so a press over a drone would otherwise reselect mid-orbit.
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                click_origin = GetMousePosition();
                click_in_view = GetMouseY() <
                    GetScreenHeight() - hud_bar_height(&hud, GetScreenHeight());
            }
            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && click_in_view &&
                Vector2Distance(click_origin, GetMousePosition()) < CLICK_DRAG_SLOP_PX) {
                Ray ray = GetMouseRay(GetMousePosition(), scene.camera);
                float nearest = INFINITY;
                int hit = -1;
                for (int i = 0; i < vehicle_count; i++) {
                    if (!vehicles[i].active) continue;
                    float radius = (i == selected || vehicle_count <= 16)
                                     ? vehicles[i].model_scale : 0.35f;
                    RayCollision collision = GetRayCollisionSphere(
                        ray, vehicles[i].position, fmaxf(radius, 0.25f));
                    if (collision.hit && collision.distance < nearest) {
                        nearest = collision.distance;
                        hit = i;
                    }
                }
                if (hit >= 0) {
                    selected = hit;
                    hud.selector_page = selected / FLEET_PAGE_SIZE;
                    hud.pinned_count = 0;
                    memset(hud.pinned, -1, sizeof(hud.pinned));
                    chord_value = -1;
                }
            }
            if (IsKeyPressed(KEY_TAB)) {
                // Cycle to next connected vehicle, clear pins
                for (int j = 1; j <= vehicle_count; j++) {
                    int next = (selected + j) % vehicle_count;
                    if (sources[next].connected) { selected = next; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
                chord_value = -1;
                hud.selector_page = selected / FLEET_PAGE_SIZE;
            }
            // PageUp/PageDown scroll the selector, in replay as well as live —
            // [ and ] stay on marker cycling, as docs/keybinds.md documents.
            if ((IsKeyPressed(KEY_PAGE_UP) || IsKeyPressed(KEY_PAGE_DOWN)) &&
                vehicle_count > FLEET_PAGE_SIZE) {
                int pages = (vehicle_count + FLEET_PAGE_SIZE - 1) / FLEET_PAGE_SIZE;
                int dir = IsKeyPressed(KEY_PAGE_UP) ? -1 : 1;
                hud.selector_page = (hud.selector_page + dir + pages) % pages;
                chord_value = -1;
            }
            // Number chords accept every digit required by the runtime fleet size.
            {
                int digit = -1;
                for (int k = KEY_ZERO; k <= KEY_NINE; k++) {
                    if (IsKeyPressed(k)) { digit = k - KEY_ZERO; break; }
                }

                bool shift_held = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
                bool ctrl_held = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
                bool alt_held = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT);

                // Check chord timeout
                if (chord_value >= 0 && GetTime() - chord_time > CHORD_TIMEOUT_S) {
                    int idx = chord_value - 1;  // digit 1 = drone index 0
                    if (idx >= 0 && idx < vehicle_count) {
                        apply_vehicle_selection_hud(&hud, idx, chord_shift, &selected, vehicle_count);
                        hud.selector_page = selected / FLEET_PAGE_SIZE;
                    }
                    chord_value = -1;
                }

                if (digit >= 0 && !ctrl_held && !alt_held) {
                    if (chord_value < 0 && digit > 0) {
                        chord_value = digit;
                        chord_shift = shift_held;
                    } else if (chord_value >= 0 && chord_value <= VEHICLE_SANITY_LIMIT / 10) {
                        chord_value = chord_value * 10 + digit;
                    }
                    chord_time = GetTime();
                    // apply_vehicle_selection() does not range-check, and every
                    // per-vehicle array is sized to exactly vehicle_count.
                    if (vehicle_count <= 9 && chord_value > 0) {
                        if (chord_value <= vehicle_count)
                            apply_vehicle_selection_hud(&hud, chord_value - 1, chord_shift,
                                                        &selected, vehicle_count);
                        hud.selector_page = selected / FLEET_PAGE_SIZE;
                        chord_value = -1;
                    }
                }
            }
        }
        } // end !marker_input.active guard

        // Marker label text input — consumes all keyboard while active
        if (marker_input.active) {
            int di = marker_input.drone_idx;
            marker_input_update(&marker_input, &markers[di], &sources[di], &vehicles[di]);
        }

        // Re-toast insufficient data warning when switching to a bad drone
        {
            static int last_selected_for_insuf = -1;
            if (selected != last_selected_for_insuf && insufficient_data[selected]) {
                char msg[64];
                snprintf(msg, sizeof(msg), "DRONE %d: INSUFFICIENT DATA", selected + 1);
                hud_toast_color(&hud, msg, 3.0f, vehicles[selected].color);
            }
            last_selected_for_insuf = selected;
        }

        // Replay playback controls (apply to all sources)
        if (is_replay && !marker_input.active) {
            bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
            int nrf = num_replay_files > 0 ? num_replay_files : 1;
            if (IsKeyPressed(KEY_SPACE)) {
                if (!sources[selected].connected) {
                    for (int i = 0; i < nrf; i++) {
                        data_source_seek(&sources[i], 0.0f);
                        sources[i].connected = true;
                        sources[i].playback.paused = false;
                        vehicle_reset_trail(&vehicles[i]);
                    }
                    memset(corr, 0, (size_t)vehicle_count * sizeof(*corr));
                } else {
                    bool p = !sources[selected].playback.paused;
                    for (int i = 0; i < nrf; i++)
                        sources[i].playback.paused = p;
                }
            }
            if (IsKeyPressed(KEY_L) && !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL) && shift) {
                bool l = !sources[selected].playback.looping;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.looping = l;
            }
            if (IsKeyPressed(KEY_Y)) {
                hud.show_yaw = !hud.show_yaw;
            }
            if (IsKeyPressed(KEY_N)) {
                hud.show_notifications = !hud.show_notifications;
                hud_toast(&hud, hud.show_notifications ? "Notifications On" : "Notifications Off", 2.0f);
            }
            if (IsKeyPressed(KEY_L) && !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL) && !shift && (markers[selected].last_drop_idx < 0 || GetTime() - markers[selected].last_drop_time >= 0.5)) {
                show_marker_labels = !show_marker_labels;
            }
            if (IsKeyPressed(KEY_I)) {
                bool interp = !sources[selected].playback.interpolation;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.interpolation = interp;
                hud_toast(&hud, interp ? "Interpolation On" : "Interpolation Off", 2.0f);
            }
            if (IsKeyPressed(KEY_EQUAL)) {
                float spd = sources[selected].playback.speed;
                if (spd < 0.5f) spd = 0.5f;
                else if (spd < 1.0f) spd = 1.0f;
                else if (spd < 2.0f) spd = 2.0f;
                else if (spd < 4.0f) spd = 4.0f;
                else if (spd < 8.0f) spd = 8.0f;
                else spd = 16.0f;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.speed = spd;
            }
            if (IsKeyPressed(KEY_MINUS)) {
                float spd = sources[selected].playback.speed;
                if (spd > 8.0f) spd = 8.0f;
                else if (spd > 4.0f) spd = 4.0f;
                else if (spd > 2.0f) spd = 2.0f;
                else if (spd > 1.0f) spd = 1.0f;
                else if (spd > 0.5f) spd = 0.5f;
                else spd = 0.25f;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.speed = spd;
            }
            if (IsKeyPressed(KEY_R)) {
                for (int i = 0; i < nrf; i++) {
                    data_source_seek(&sources[i], 0.0f);
                    sources[i].connected = true;
                    sources[i].playback.paused = false;
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, (size_t)vehicle_count * sizeof(*corr));
                for (int i = 0; i < vehicle_count; i++) {
                    markers[i].count = 0;
                    markers[i].current = -1;
                }
            }
            // A key: toggle takeoff time alignment
            if (IsKeyPressed(KEY_A) && num_replay_files > 1) {
                takeoff_aligned = !takeoff_aligned;
                const float takeoff_buffer = 5.0f;
                for (int i = 0; i < nrf; i++) {
                    if (takeoff_aligned) {
                        float skip = sources[i].playback.takeoff_detected
                            ? sources[i].playback.takeoff_time_s - takeoff_buffer : 0.0f;
                        if (skip < 0.0f) skip = 0.0f;
                        data_source_set_time_offset(&sources[i], (double)skip);
                    } else {
                        data_source_set_time_offset(&sources[i], 0.0);
                    }
                    data_source_seek(&sources[i], 0.0f);
                    sources[i].connected = true;
                    sources[i].playback.paused = false;
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, (size_t)vehicle_count * sizeof(*corr));
                for (int i = 0; i < nrf; i++) {
                    sources[i].playback.correlation = NAN;
                    sources[i].playback.rmse = NAN;
                }
                // Re-resolve system marker positions with new time offsets
                for (int i = 0; i < nrf; i++)
                    sys_markers[i].resolved = false;
                hud_toast(&hud, takeoff_aligned ? "Auto Align On" : "Auto Align Off", 2.0f);
            }

            // Timeline scrubbing: 3 levels of granularity
            // Shift+Arrow = single frame step (~20ms), Ctrl+Shift = 1s, plain = 5s
            if (IsKeyPressed(KEY_RIGHT)) {
                float step;
                if (shift && ctrl) step = 1.0f;
                else if (shift) { step = 0.02f; sources[selected].playback.paused = true; }
                else step = 5.0f;
                float seek_target = sources[selected].playback.position_s + step;
                for (int i = 0; i < nrf; i++) {
                    data_source_seek(&sources[i], seek_target);
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, (size_t)vehicle_count * sizeof(*corr));
            }
            if (IsKeyPressed(KEY_LEFT)) {
                float step;
                if (shift && ctrl) step = 1.0f;
                else if (shift) { step = 0.02f; sources[selected].playback.paused = true; }
                else step = 5.0f;
                float target = sources[selected].playback.position_s - step;
                if (target < 0.0f) target = 0.0f;
                for (int i = 0; i < nrf; i++) {
                    data_source_seek(&sources[i], target);
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, (size_t)vehicle_count * sizeof(*corr));
            }

            // Frame markers: B = drop marker, B->L = drop + label, Shift+B = delete current
            if (IsKeyPressed(KEY_B) && vehicles[selected].active && !marker_input.active) {
                if (shift) {
                    marker_delete(&markers[selected]);
                } else if (markers[selected].count < REPLAY_MAX_MARKERS) {
                    marker_drop(&markers[selected], sources[selected].playback.position_s,
                                vehicles[selected].position, &vehicles[selected],
                                &sys_markers[selected]);
                }
            }

            // B->L chord: if L pressed within 0.5s of dropping a marker, open label input
            if (IsKeyPressed(KEY_L) && !marker_input.active
                && markers[selected].last_drop_idx >= 0) {
                double elapsed = GetTime() - markers[selected].last_drop_time;
                if (elapsed < 0.5) {
                    marker_input_begin(&marker_input, markers[selected].last_drop_idx,
                                       &sources[selected]);
                    marker_input.drone_idx = selected;
                    markers[selected].last_drop_idx = -1;
                    // Pause all drones during label input
                    for (int i = 0; i < vehicle_count; i++)
                        sources[i].playback.paused = true;
                }
            }

            // [/] cycling: per-drone or global (Ctrl)
            if (IsKeyPressed(KEY_LEFT_BRACKET) || IsKeyPressed(KEY_RIGHT_BRACKET)) {
                int dir = IsKeyPressed(KEY_LEFT_BRACKET) ? -1 : 1;
                bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
                if (ctrl) {
                    marker_cycle_result_t r = marker_cycle_global(
                        markers, sys_markers, vehicle_count, selected, dir,
                        sources, vehicles, precomp, &scene, last_pos);
                    if (r.jumped && r.drone_idx != selected)
                        selected = r.drone_idx;
                } else if (markers[selected].count > 0 || sys_markers[selected].count > 0) {
                    marker_cycle(&markers[selected], &sys_markers[selected], dir, shift,
                                 &sources[selected], &vehicles[selected],
                                 &precomp[selected], &scene, &last_pos[selected]);
                }
                // Trigger annunciators on marker cycle
                if (!shift) {
                    annunc_trigger_tab_fade(&hud.annunciators, selected);
                    annunc_trigger_radar_wave(&hud.annunciators, selected);
                    // Bounce any pinned drones
                    for (int p = 0; p < hud.pinned_count; p++) {
                        int pidx = hud.pinned[p];
                        if (pidx >= 0 && pidx < vehicle_count)
                            annunc_trigger_ring_bounce(&hud.annunciators, pidx);
                    }
                }
                // Sync all drones to the same playback time after marker seek
                if (!shift && vehicle_count > 1) {
                    float sync_time = sources[selected].playback.position_s;
                    for (int i = 0; i < vehicle_count; i++) {
                        if (i == selected) continue;
                        data_source_seek(&sources[i], sync_time);
                        vehicle_reset_trail(&vehicles[i]);
                    }
                }
            }
        }

        // Update debug panel
        debug_panel_update(&dbg_panel, GetFrameTime());

        // Update camera to follow selected vehicle
        {
            Vector3 cam_target = vehicles[selected].position;
            if (hud.mode == HUD_TACTICAL)
                cam_target.y += scene.chase_distance * 0.10f;

            // An unattended run frames the map rather than the aircraft. The
            // two are not the same place: a vehicle whose origin has not
            // resolved yet is drawn at the world origin while its rays are
            // already landing wherever the fleet frame says they belong.
            if (follow_map && map_ready) {
                double lo[3], hi[3];
                if (octomap_content_bounds(&map_session.map, lo, hi)) {
                    const double mid[3] = { (lo[0] + hi[0]) * 0.5,
                                            (lo[1] + hi[1]) * 0.5,
                                            (lo[2] + hi[2]) * 0.5 };
                    float w[3];
                    fleet_enu_to_world(mid, w);
                    cam_target = (Vector3){ w[0], w[1], w[2] };

                    if (view_span_m > 0.0) {
                        scene.ortho_span = (float)view_span_m;
                    } else {
                        // Fit, with a margin so the newest cells are not sitting
                        // on the edge of frame the moment they appear.
                        double span = hi[0] - lo[0];
                        if (hi[1] - lo[1] > span) span = hi[1] - lo[1];
                        if (hi[2] - lo[2] > span) span = hi[2] - lo[2];
                        span *= 1.25;
                        if (span < 10.0) span = 10.0;
                        if (span > 500.0) span = 500.0;
                        // Ease rather than snap: the map grows chunk by chunk and
                        // a camera that jumps on every new one is unwatchable.
                        scene.ortho_span += ((float)span - scene.ortho_span) * 0.08f;
                    }
                    scene.ortho_pan = (Vector3){ 0, 0, 0 };
                    scene.chase_distance = scene.ortho_span * 0.9f;
                }
            }
            scene_update_camera(&scene, cam_target, vehicles[selected].rotation);
        }

        // Render ortho views to textures (before main BeginDrawing)
        // Always render when tactical HUD is active (radar uses top-down data)
        if (ortho.visible || hud.mode == HUD_TACTICAL) {
            ortho_panel_update(&ortho, vehicles[selected].position);
            ortho_panel_render(&ortho, vehicles, vehicle_count,
                               selected, scene.theme,
                               corr_mode, hud.pinned, hud.pinned_count);
        }

        // Render
        BeginDrawing();

            // Sky background
            scene_draw_sky(&scene);

            // Fullscreen ortho: suppress 3D trails, conditionally suppress 3D correlation line
            bool fs_ortho = (scene.ortho_mode != ORTHO_NONE);
            int tm_3d = fs_ortho ? 0 : trail_mode;

            BeginMode3D(scene.camera);
                scene_draw(&scene);

                // The shared map goes down before the vehicles so translucent
                // free space does not wash the models out.
                if (map_render_ready)
                    map_render_draw(&map_render, &map_session, scene.camera, scene.theme);
                if (map_ready)
                    quality_overlay_draw_3d(&map_session, &quality_opts, selected, scene.theme);
                if (vehicle_count > 60) {
                    draw_density_heatmap(vehicles, vehicle_count, scene.theme);
                    // rlgl has no RL_POINTS primitive; DrawPoint3D is raylib's
                    // point, batched into the same RL_LINES draw call.
                    for (int i = 0; i < vehicle_count; i++) {
                        if (i == selected || !vehicles[i].active) continue;
                        DrawPoint3D(vehicles[i].position, vehicles[i].color);
                    }
                }
                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicles[i].active || vehicle_count == 1) {
                        if (vehicle_count <= 16 || i == selected) {
                            vehicle_draw(&vehicles[i], scene.theme, i == selected,
                                         (i == selected || vehicle_count <= 16) ? tm_3d : 0,
                                         show_ground_track, scene.camera.position,
                                         classic_colors);
                        } else if (vehicle_count <= 60) {
                            DrawSphere(vehicles[i].position, 0.18f, vehicles[i].color);
                        }
                    }
                }
                // Draw frame marker spheres and system marker cubes for all drones
                for (int i = 0; i < vehicle_count; i++) {
                    if (is_replay && markers[i].count > 0) {
                        int cur = (i == selected && !sys_markers[i].selected)
                                  ? markers[i].current : -1;
                        vehicle_draw_markers(markers[i].positions, markers[i].labels,
                                             markers[i].count, cur,
                                             scene.camera.position, scene.camera,
                                             markers[i].roll, markers[i].pitch,
                                             markers[i].vert, markers[i].speed,
                                             vehicles[i].trail_speed_max, scene.theme,
                                             trail_mode, MARKER_USER,
                                             vehicles[i].color);
                    }
                    if (is_replay && sys_markers[i].count > 0) {
                        int cur = (i == selected && sys_markers[i].selected)
                                  ? sys_markers[i].current : -1;
                        vehicle_draw_markers(sys_markers[i].positions, sys_markers[i].labels,
                                             sys_markers[i].count, cur,
                                             scene.camera.position, scene.camera,
                                             sys_markers[i].roll, sys_markers[i].pitch,
                                             sys_markers[i].vert, sys_markers[i].speed,
                                             vehicles[i].trail_speed_max, scene.theme,
                                             trail_mode, MARKER_SYSTEM,
                                             vehicles[i].color);
                    }
                }

                // Home position markers (formation mode only)
                if (num_replay_files > 1 && !ghost_mode && !ghost_mode_grid) {
                    for (int i = 0; i < vehicle_count; i++) {
                        if (!sources[i].home.valid) continue;
                        double lat = sources[i].home.lat * 1e-7 * (M_PI / 180.0);
                        double lon = sources[i].home.lon * 1e-7 * (M_PI / 180.0);
                        double alt = sources[i].home.alt * 1e-3;
                        float hx = (float)(EARTH_RADIUS * (lon - ref_lon_rad) * cos(ref_lat_rad)) + vehicles[i].grid_offset.x;
                        float hy = (float)(alt - min_alt) + 0.02f;
                        float hz = (float)(-(EARTH_RADIUS * (lat - ref_lat_rad))) + vehicles[i].grid_offset.z;
                        float half = 0.333f;
                        Color fill = vehicles[i].color;
                        fill.a = 70;
                        Color border = vehicles[i].color;
                        border.a = 180;
                        DrawPlane((Vector3){hx, hy, hz}, (Vector2){half * 2, half * 2}, fill);
                        DrawLine3D((Vector3){hx - half, hy, hz - half}, (Vector3){hx + half, hy, hz - half}, border);
                        DrawLine3D((Vector3){hx + half, hy, hz - half}, (Vector3){hx + half, hy, hz + half}, border);
                        DrawLine3D((Vector3){hx + half, hy, hz + half}, (Vector3){hx - half, hy, hz + half}, border);
                        DrawLine3D((Vector3){hx - half, hy, hz + half}, (Vector3){hx - half, hy, hz - half}, border);
                    }
                }

                // Axis gizmo at selected drone (Z key toggle)
                if (show_axes && vehicles[selected].active) {
                    Vector3 com = vehicles[selected].position;
                    com.y += vehicles[selected].model_scale * 0.15f;
                    draw_axis_gizmo_3d(com, vehicles[selected].model_scale * 0.5f,
                                        vehicles[selected].rotation);
                }

                // Correlation overlay: line (mode 1) or curtain (mode 2)
                // In fullscreen ortho, skip corr_mode==1 (line) — drawn in 2D instead.
                // Curtain (corr_mode==2) always stays in 3D.
                if (corr_mode > 0 && hud.pinned_count > 0) {
                    for (int p = 0; p < hud.pinned_count; p++) {
                        int pidx = hud.pinned[p];
                        if (pidx >= 0 && pidx < vehicle_count && vehicles[pidx].active
                            && pidx != selected) {
                            if (corr_mode == 1 && !fs_ortho) {
                                vehicle_draw_correlation_line(
                                    &vehicles[selected], &vehicles[pidx]);
                            } else if (corr_mode == 2) {
                                vehicle_draw_correlation_curtain(
                                    &vehicles[selected], &vehicles[pidx],
                                    scene.theme, scene.camera.position);
                            }
                        }
                    }
                }
            EndMode3D();

            // Colour repeats after the finite palette; the runtime index never does.
            if (vehicle_count > 16) {
                Vector3 label_cam_fwd = Vector3Normalize(Vector3Subtract(
                    scene.camera.target, scene.camera.position));
                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicle_count > 60 && i != selected) continue;
                    if (!vehicles[i].active) continue;
                    // GetWorldToScreen mirrors points behind the camera onto
                    // the viewport; skip them rather than draw phantom labels.
                    if (Vector3DotProduct(Vector3Subtract(vehicles[i].position,
                                                          scene.camera.position),
                                          label_cam_fwd) <= 0.0f) continue;
                    Vector2 p = GetWorldToScreen(vehicles[i].position, scene.camera);
                    char label[16];
                    snprintf(label, sizeof(label), "%d", i + 1);
                    float fs = (i == selected) ? 16.0f : 11.0f;
                    Vector2 size = MeasureTextEx(hud.font_label, label, fs, 0.5f);
                    DrawTextEx(hud.font_label, label,
                               (Vector2){p.x - size.x / 2, p.y - 18}, fs, 0.5f,
                               i == selected ? WHITE : vehicles[i].color);
                }
            }

            // Ortho ground fill (2D overlay)
            scene_draw_ortho_ground(&scene, GetScreenWidth(), GetScreenHeight());

            // Screen edge indicators for off-screen drones
            if (vehicle_count > 1 && show_edge_indicators) {
                float ei_scale = powf(GetScreenHeight() / 720.0f, 0.7f);
                if (ei_scale < 1.0f) ei_scale = 1.0f;
                draw_edge_indicators(vehicles, vehicle_count, selected,
                                     scene.camera, hud.font_value, ei_scale);
            }

            // Marker labels (2D billboarded text, after EndMode3D)
            if (is_replay && show_marker_labels) {
                for (int i = 0; i < vehicle_count; i++) {
                    if (markers[i].count > 0) {
                        int cur = (i == selected && !sys_markers[i].selected)
                                  ? markers[i].current : -1;
                        vehicle_draw_marker_labels(markers[i].positions, markers[i].labels,
                                                   markers[i].count, cur,
                                                   scene.camera.position, scene.camera,
                                                   hud.font_label, hud.font_value,
                                                   markers[i].roll, markers[i].pitch,
                                                   markers[i].vert, markers[i].speed,
                                                   vehicles[i].trail_speed_max, scene.theme,
                                                   trail_mode, MARKER_USER,
                                                   vehicles[i].color);
                    }
                    if (sys_markers[i].count > 0) {
                        int cur = (i == selected && sys_markers[i].selected)
                                  ? sys_markers[i].current : -1;
                        vehicle_draw_marker_labels(sys_markers[i].positions, sys_markers[i].labels,
                                                   sys_markers[i].count, cur,
                                                   scene.camera.position, scene.camera,
                                                   hud.font_label, hud.font_value,
                                                   sys_markers[i].roll, sys_markers[i].pitch,
                                                   sys_markers[i].vert, sys_markers[i].speed,
                                                   vehicles[i].trail_speed_max, scene.theme,
                                                   trail_mode, MARKER_SYSTEM,
                                                   vehicles[i].color);
                    }
                }
            }

            // Fullscreen ortho 2D overlays (trails + correlation line)
            ortho_draw_fullscreen_2d(&scene, vehicles, vehicle_count,
                                      selected, trail_mode,
                                      corr_mode, hud.pinned, hud.pinned_count,
                                      GetScreenWidth(), GetScreenHeight(),
                                      hud.font_label, show_corr_labels);

            // HUD
            if (show_hud) {
                bool has_awaiting_gps = vehicles[selected].active &&
                    !vehicles[selected].origin_set && sources[selected].home.valid;
                for (int i = 0; i < vehicle_count; i++) {
                    all_user_md[i] = (hud_marker_data_t){
                        .times = markers[i].times,
                        .labels = markers[i].labels,
                        .roll = markers[i].roll,
                        .pitch = markers[i].pitch,
                        .vert = markers[i].vert,
                        .speed = markers[i].speed,
                        .speed_max = markers[i].speed_max,
                        .count = markers[i].count,
                        .current = markers[i].current,
                        .selected = true,
                        .color = vehicles[i].color,
                    };
                    all_sys_md[i] = (hud_marker_data_t){
                        .times = sys_markers[i].times,
                        .labels = sys_markers[i].labels,
                        .roll = sys_markers[i].roll,
                        .pitch = sys_markers[i].pitch,
                        .vert = sys_markers[i].vert,
                        .speed = sys_markers[i].speed,
                        .speed_max = markers[i].speed_max,
                        .count = sys_markers[i].count,
                        .current = sys_markers[i].current,
                        .selected = sys_markers[i].selected,
                        .color = vehicles[i].color,
                    };
                }
                if (hud.mode == HUD_CONSOLE) {
                    hud_draw(&hud, vehicles, sources, vehicle_count,
                             selected, GetScreenWidth(), GetScreenHeight(),
                             scene.theme, trail_mode,
                             all_user_md, all_sys_md, vehicle_count,
                             ghost_mode, has_tier3, has_awaiting_gps);
                } else if (hud.mode == HUD_TACTICAL) {
                    tactical_hud_draw(&hud, vehicles, sources, vehicle_count,
                                      selected, GetScreenWidth(), GetScreenHeight(),
                                      scene.theme, ghost_mode,
                                      has_tier3, has_awaiting_gps,
                                      &ortho, trail_mode, corr_mode,
                                      all_user_md, all_sys_md, vehicle_count);
                }
            }

            // Debug panel
            {
                int active_count = 0;
                int total_trail = 0;
                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicles[i].active) active_count++;
                    total_trail += vehicles[i].trail_count;
                }
                debug_panel_draw(&dbg_panel, GetScreenWidth(), GetScreenHeight(),
                                 scene.theme, hud.font_label,
                                 vehicle_count, active_count, total_trail,
                                 vehicles[selected].position,
                                 sources[selected].ref_rejected,
                                 vehicle_tier[selected]);
            }

            // Ortho panel overlay (sidebar in Console mode; tactical draws its own insets)
            if (hud.mode != HUD_TACTICAL) {
                int bar_h = show_hud ? hud_bar_height(&hud, GetScreenHeight()) : 0;
                ortho_panel_draw(&ortho, GetScreenHeight(), bar_h, scene.theme, hud.font_label,
                                 vehicles, vehicle_count, selected, trail_mode,
                                 corr_mode, hud.pinned, hud.pinned_count,
                                 show_axes);
            }

            // Fullscreen ortho view label
            ortho_panel_draw_fullscreen_label(GetScreenWidth(), GetScreenHeight(),
                scene.ortho_mode, scene.ortho_span, scene.theme, hud.font_label,
                show_axes);

            // Marker label input overlay (view-mode-aware)
            if (marker_input.active) {
                marker_input_draw(&marker_input, hud.font_label, hud.font_value,
                                  scene.theme, GetScreenWidth(), GetScreenHeight());
            }

            // Map HUD. At swarm scale the map is the point and per-vehicle
            // detail is not, so the panel collapses to one line above 16.
            if (map_ready) {
                const int sw2 = GetScreenWidth(), sh2 = GetScreenHeight();
                if (show_map_panel && vehicle_count <= 16) {
                    const int pw = 232;
                    int py = 96;
                    py += map_hud_draw_panel(&map_session, &map_render, sw2 - pw - 12, py,
                                             pw, hud.font_value, scene.theme) + 8;
                    quality_overlay_draw_panel(&map_session, &quality_opts, selected,
                                               sw2 - pw - 12, py, pw, hud.font_value,
                                               scene.theme);
                } else if (show_map_panel) {
                    map_hud_draw_compact(&map_session, &map_render, 12, sh2 - 96,
                                         hud.font_value);
                }
                if (map_hud_opts.show_timeline) {
                    const int th = (int)map_hud_opts.timeline_height;
                    map_hud_draw_timeline(&map_session, 12, sh2 - th - 12,
                                          sw2 - 24, th, hud.font_value, scene.theme);
                }
            }

        EndDrawing();

        // After the present, so what is recorded is exactly the frame that was
        // shown -- there is no second offscreen path that could drift from it.
        if (capture_tick(&capture)) break;
    }

    capture_finish(&capture);

    // Cleanup
    if (map_render_ready) map_render_free(&map_render);
    if (map_ready) map_session_free(&map_session);
    ortho_panel_cleanup(&ortho);
    hud_cleanup(&hud);
    for (int i = 0; i < vehicle_count; i++) {
        vehicle_cleanup(&vehicles[i]);
        if (sources[i].ops) data_source_close(&sources[i]);
    }
    scene_cleanup(&scene);
    for (int i = 0; i < vehicle_count; i++)
        precomp_trail_cleanup(&precomp[i]);
    free(replay_paths);
    free(sources); free(vehicles); free(corr); free(vehicle_tier);
    free(was_connected); free(last_pos); free(insufficient_data);
    free(prev_playback_pos); free(markers); free(sys_markers); free(precomp);
    free(all_user_md); free(all_sys_md);
    CloseWindow();

    return 0;
}
