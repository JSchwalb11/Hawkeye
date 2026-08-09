#ifndef MAP_HUD_H
#define MAP_HUD_H

// The map's own HUD: what the map knows, what it cost, and how the sources were
// aligned in time.
//
// Two things here are not decoration. The time provenance is shown because an
// unlabelled alignment guess is a lie a viewer tells quietly. The ray-drop
// counters are shown because a map that silently falls behind looks exactly
// like a map of an empty room.

#include <stdbool.h>

#include "raylib.h"
#include "map_render.h"
#include "map_session.h"
#include "theme.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool  show_panel;
    bool  show_timeline;
    float timeline_height;
} map_hud_opts_t;

void map_hud_defaults(map_hud_opts_t *o);

// The stats/legend panel. Returns the height consumed.
int map_hud_draw_panel(const map_session_t *ms, const map_render_t *mr,
                       int x, int y, int width, Font font, const theme_t *theme);

// The timeline's pointer handling, split out from the drawing so it can run
// before the camera sees the same mouse button. Call it once per frame with
// the rectangle the widget will be drawn into, ahead of scene_handle_input;
// it returns true when the pointer is over the widget, which is the caller's
// cue to keep the camera's hands off it.
bool map_hud_timeline_input(map_session_t *ms, int x, int y, int width, int height);

// The replay timeline: playhead, keyframes, event marks and the live pin.
// Drawing only -- map_hud_timeline_input does the interacting.
void map_hud_draw_timeline(map_session_t *ms, int x, int y, int width, int height,
                           Font font, const theme_t *theme);

// One-line summary for the corner of a crowded screen.
void map_hud_draw_compact(const map_session_t *ms, const map_render_t *mr,
                          int x, int y, Font font);

#ifdef __cplusplus
}
#endif

#endif
