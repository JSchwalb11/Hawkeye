#include "map_hud.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define MH_LINE 15
#define MH_PAD  8

#define COL_TEXT   (Color){ 230, 237, 243, 255 }
#define COL_DIM    (Color){ 139, 152, 165, 235 }
#define COL_BG     (Color){ 14, 17, 22, 215 }
#define COL_EDGE   (Color){ 42, 51, 65, 255 }
#define COL_OK     (Color){ 123, 228, 149, 255 }
#define COL_WARN   (Color){ 255, 209, 102, 255 }
#define COL_BAD    (Color){ 255, 99, 132, 255 }
#define COL_LIVE   (Color){ 255, 79, 163, 255 }

void map_hud_defaults(map_hud_opts_t *o) {
    if (!o) return;
    o->show_panel = true;
    o->show_timeline = true;
    o->timeline_height = 54.0f;
}

static void row(Font f, int x, int *y, const char *label, const char *value, Color vc) {
    DrawTextEx(f, label, (Vector2){ (float)x, (float)*y }, 12, 1.0f, COL_DIM);
    DrawTextEx(f, value, (Vector2){ (float)(x + 118), (float)*y }, 12, 1.0f, vc);
    *y += MH_LINE;
}

static void swatch_row(Font f, int x, int *y, Color sw, const char *text) {
    DrawRectangle(x, *y + 2, 9, 9, sw);
    DrawRectangleLines(x, *y + 2, 9, 9, COL_EDGE);
    DrawTextEx(f, text, (Vector2){ (float)(x + 15), (float)*y }, 12, 1.0f, COL_DIM);
    *y += MH_LINE;
}

static void human_bytes(char *out, size_t n, size_t bytes) {
    if (bytes >= (1u << 30)) snprintf(out, n, "%.2f GiB", bytes / 1073741824.0);
    else if (bytes >= (1u << 20)) snprintf(out, n, "%.1f MiB", bytes / 1048576.0);
    else snprintf(out, n, "%.0f KiB", bytes / 1024.0);
}

int map_hud_draw_panel(const map_session_t *ms, const map_render_t *mr,
                       int x, int y, int width, Font font, const theme_t *theme) {
    (void)theme;
    if (!ms || !mr) return 0;

    const int height = 268;
    DrawRectangle(x, y, width, height, COL_BG);
    DrawRectangleLines(x, y, width, height, COL_EDGE);

    char buf[128];
    int ty = y + MH_PAD;
    snprintf(buf, sizeof(buf), "FLEET MAP  %s", map_draw_mode_name(mr->mode));
    DrawTextEx(font, buf, (Vector2){ (float)(x + MH_PAD), (float)ty }, 13, 1.0f, COL_TEXT);
    ty += 20;

    const int lx = x + MH_PAD;
    const map_ingest_stats_t *st = map_ingest_stats(&ms->ingest);

    // Time alignment first: everything downstream is only as good as this.
    const time_provenance_t prov = map_session_time_provenance(ms);
    const int64_t spread = map_session_time_spread_ns(ms);
    Color prov_col = COL_OK;
    if (prov <= TIME_PROV_BOOT_ASSUMED) prov_col = COL_BAD;
    else if (prov <= TIME_PROV_TLOG_ARRIVAL) prov_col = COL_WARN;
    row(font, lx, &ty, "TIME SOURCE", timebase_provenance_name(prov), prov_col);
    snprintf(buf, sizeof(buf), "%.3f s", (double)spread * 1e-9);
    row(font, lx, &ty, "FLEET SPREAD", buf, fabs((double)spread) > 5e8 ? COL_WARN : COL_TEXT);

    const om_stats_t *os = &ms->map.stats;
    snprintf(buf, sizeof(buf), "%llu / %llu", (unsigned long long)os->hits_inserted,
             (unsigned long long)os->misses_inserted);
    row(font, lx, &ty, "HITS / NO-RET", buf, COL_TEXT);

    snprintf(buf, sizeof(buf), "%.0f/s  q %u/%u", st->rays_per_s,
             st->queue_depth, st->queue_capacity);
    row(font, lx, &ty, "RAY RATE", buf, COL_TEXT);

    // Dropped rays are reported, never absorbed: an operator has to be able to
    // tell "nothing there" from "we stopped looking".
    snprintf(buf, sizeof(buf), "%llu  (%.0f/s)", (unsigned long long)st->dropped,
             st->drops_per_s);
    row(font, lx, &ty, "RAYS DROPPED", buf,
        st->dropped ? (map_ingest_overloaded(&ms->ingest) ? COL_BAD : COL_WARN) : COL_TEXT);

    const uint32_t live_nodes = ms->map.node_count - ms->map.free_blocks * 8;
    char mem[32];
    human_bytes(mem, sizeof(mem), octomap_bytes(&ms->map));
    snprintf(buf, sizeof(buf), "%s  %u nodes", mem, live_nodes);
    row(font, lx, &ty, "MEMORY", buf, COL_TEXT);

    snprintf(buf, sizeof(buf), "%llu  refused %llu", (unsigned long long)os->prunes,
             (unsigned long long)os->alloc_refusals);
    row(font, lx, &ty, "PRUNED", buf, os->alloc_refusals ? COL_WARN : COL_TEXT);

    snprintf(buf, sizeof(buf), "%u/%u chunks  %u inst",
             mr->stats.chunks_drawn, mr->stats.chunks_live, mr->stats.instances_drawn);
    row(font, lx, &ty, "RENDER", buf, COL_TEXT);

    ty += 4;
    switch (mr->mode) {
        case MAP_DRAW_COVERAGE:
            swatch_row(font, lx, &ty, (Color){  46, 111, 142, 200 }, "observed, free");
            swatch_row(font, lx, &ty, (Color){ 255, 180,  84, 235 }, "observed, occupied");
            swatch_row(font, lx, &ty, COL_BG, "unknown - never looked");
            break;
        case MAP_DRAW_DIVERGENCE:
            swatch_row(font, lx, &ty, (Color){ 255,  79, 163, 245 }, "contested - fleet disagrees");
            swatch_row(font, lx, &ty, (Color){ 176, 122,  58, 200 }, "agreed occupancy");
            break;
        case MAP_DRAW_CONTRIBUTION:
            snprintf(buf, sizeof(buf), "vehicle %d only", ms->focus_vehicle);
            swatch_row(font, lx, &ty, (Color){  79, 195, 255, 240 }, buf);
            swatch_row(font, lx, &ty, (Color){ 240, 246, 252, 220 }, "focus and others");
            swatch_row(font, lx, &ty, (Color){ 176, 122,  58, 200 }, "others only");
            break;
        default:
            swatch_row(font, lx, &ty, (Color){ 255, 180,  84, 235 }, "occupied");
            swatch_row(font, lx, &ty, (Color){  46, 111, 142, 200 }, "free (carved)");
            swatch_row(font, lx, &ty, COL_BG, "unknown");
            break;
    }
    return height;
}

void map_hud_draw_compact(const map_session_t *ms, const map_render_t *mr,
                          int x, int y, Font font) {
    if (!ms || !mr) return;
    const map_ingest_stats_t *st = map_ingest_stats(&ms->ingest);
    char buf[160];
    snprintf(buf, sizeof(buf), "MAP %s  %.0f ray/s  drop %llu  %s",
             map_draw_mode_name(mr->mode), st->rays_per_s,
             (unsigned long long)st->dropped,
             timebase_provenance_name(map_session_time_provenance(ms)));
    DrawTextEx(font, buf, (Vector2){ (float)x, (float)y }, 12, 1.0f,
               st->dropped ? COL_WARN : COL_DIM);
}

// ---------------------------------------------------------------- timeline

static Color event_colour(uint8_t kind, uint8_t severity) {
    switch (kind) {
        case TL_EVENT_MAP_DROP:    return COL_BAD;
        case TL_EVENT_MODE_CHANGE: return (Color){  79, 195, 255, 255 };
        case TL_EVENT_MISSION_ITEM:return (Color){ 123, 228, 149, 255 };
        case TL_EVENT_COMMAND_ACK: return severity <= 4 ? COL_WARN : COL_DIM;
        case TL_EVENT_TIME_REALIGN:return (Color){ 199, 146, 234, 255 };
        default:                   return severity <= 3 ? COL_BAD
                                        : (severity <= 4 ? COL_WARN : COL_DIM);
    }
}

// The widget's geometry, shared by the input and draw passes so the two can
// never disagree about where the track is.
typedef struct { int track_x, track_y, track_w, track_h, btn_x; } tl_rect_t;

static tl_rect_t tl_layout(int x, int y, int width, int height) {
    tl_rect_t r;
    r.track_x = x + MH_PAD;
    r.track_w = width - 2 * MH_PAD - 74;
    r.track_y = y + 26;
    r.track_h = height - 34;
    r.btn_x   = x + width - MH_PAD - 64;
    return r;
}

bool map_hud_timeline_input(map_session_t *ms, int x, int y, int width, int height) {
    if (!ms) return false;
    timeline_t *tl = &ms->timeline;

    const int64_t t0 = timeline_start_ns(tl);
    const int64_t t1 = timeline_head_ns(tl);
    const double span = (double)(t1 - t0);

    const tl_rect_t r = tl_layout(x, y, width, height);
    const Vector2 m = GetMousePosition();

    // Over the widget at all? Reported even when there is nothing to scrub, so
    // a click on the strip never falls through to the camera.
    const bool over = (m.x >= x && m.x <= x + width && m.y >= y && m.y <= y + height);
    if (span <= 0.0) return over;

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
        m.x >= r.btn_x && m.x <= r.btn_x + 64 &&
        m.y >= r.track_y && m.y <= r.track_y + r.track_h) {
        timeline_pin_live(tl);
    } else if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
               m.x >= r.track_x && m.x <= r.track_x + r.track_w &&
               m.y >= r.track_y - 6 && m.y <= r.track_y + r.track_h + 6) {
        double frac = (double)(m.x - r.track_x) / (double)r.track_w;
        if (frac < 0.0) frac = 0.0;
        if (frac > 1.0) frac = 1.0;
        const int64_t want = t0 + (int64_t)(frac * span);
        // A resync is a restore-and-replay. Dragging holds the button down for
        // many frames over the same pixel, so only move when the target has
        // actually changed -- otherwise the map is rebuilt for nothing.
        if (want != tl->playhead.t_ns) {
            timeline_set_playhead(tl, want);
            map_session_resync(ms);
        }
    }
    return over;
}

void map_hud_draw_timeline(map_session_t *ms, int x, int y, int width, int height,
                           Font font, const theme_t *theme) {
    (void)theme;
    if (!ms) return;
    timeline_t *tl = &ms->timeline;

    DrawRectangle(x, y, width, height, COL_BG);
    DrawRectangleLines(x, y, width, height, COL_EDGE);

    const int64_t t0 = timeline_start_ns(tl);
    const int64_t t1 = timeline_head_ns(tl);
    const double span = (double)(t1 - t0);
    if (span <= 0.0) {
        DrawTextEx(font, "TIMELINE  waiting for data",
                   (Vector2){ (float)(x + MH_PAD), (float)(y + 6) }, 12, 1.0f, COL_DIM);
        return;
    }

    const tl_rect_t r = tl_layout(x, y, width, height);
    const int track_x = r.track_x;
    const int track_w = r.track_w;
    const int track_y = r.track_y;
    const int track_h = r.track_h;

    DrawRectangle(track_x, track_y, track_w, track_h, (Color){ 24, 29, 38, 255 });

    // The stretch of history the map can actually be rebuilt across. Beyond it
    // the ray log has been evicted, and saying so beats pretending.
    const int64_t hist = timeline_map_history_start_ns(tl);
    if (hist > t0) {
        const int hx = track_x + (int)((double)(hist - t0) / span * track_w);
        DrawRectangle(track_x, track_y, hx - track_x, track_h, (Color){ 60, 30, 40, 160 });
    }

    // Keyframes: where a scrub can land without replaying from zero.
    for (uint32_t i = 0; i < tl->kf_count; i++) {
        const int kx = track_x + (int)((double)(tl->keyframes[i].t_ns - t0) / span * track_w);
        DrawRectangle(kx, track_y + track_h - 4, 1, 4, (Color){ 90, 104, 122, 255 });
    }

    // Event marks: the flight's narrative, not just its trajectory.
    for (uint32_t i = 0; i < tl->event_count; i++) {
        const uint32_t start = (tl->event_head - tl->event_count) & (tl->event_cap - 1);
        const tl_event_t *e = &tl->events[(start + i) & (tl->event_cap - 1)];
        const int ex = track_x + (int)((double)(e->t_ns - t0) / span * track_w);
        if (ex < track_x || ex > track_x + track_w) continue;
        DrawRectangle(ex, track_y, 1, track_h - 5, event_colour(e->kind, e->severity));
    }

    // Playhead.
    const int px = track_x + (int)((double)(tl->playhead.t_ns - t0) / span * track_w);
    DrawRectangle(px - 1, track_y - 3, 3, track_h + 6,
                  tl->playhead.pinned_to_head ? COL_LIVE : COL_TEXT);

    char buf[96];
    snprintf(buf, sizeof(buf), "T %+.2f s / %.1f s   %s",
             (double)(tl->playhead.t_ns - t0) * 1e-9, span * 1e-9,
             tl->playhead.pinned_to_head ? "LIVE" :
             (tl->playhead.paused ? "PAUSED" : "REPLAY"));
    DrawTextEx(font, buf, (Vector2){ (float)track_x, (float)(y + 6) }, 12, 1.0f, COL_DIM);

    if (!tl->playhead.pinned_to_head) {
        snprintf(buf, sizeof(buf), "x%.2f", (double)tl->playhead.speed);
        DrawTextEx(font, buf, (Vector2){ (float)(track_x + track_w - 40), (float)(y + 6) },
                   12, 1.0f, COL_DIM);
    }

    // The live button. Scrubbing unpins the playhead; this re-pins it, which is
    // the whole live/replay unification in one control.
    const int bx = r.btn_x;
    const int by = track_y;
    const bool hot = tl->playhead.pinned_to_head;
    DrawRectangle(bx, by, 64, track_h, hot ? (Color){ 80, 24, 50, 255 }
                                           : (Color){ 30, 36, 46, 255 });
    DrawRectangleLines(bx, by, 64, track_h, hot ? COL_LIVE : COL_EDGE);
    DrawTextEx(font, "LIVE", (Vector2){ (float)(bx + 20), (float)(by + track_h / 2 - 6) },
               12, 1.0f, hot ? COL_LIVE : COL_DIM);

}
