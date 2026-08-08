#ifndef QUALITY_OVERLAY_H
#define QUALITY_OVERLAY_H

// Quality overlays, all derived from messages that already exist on the wire.
//
// Split into a 3D part (drawn inside BeginMode3D, in world space) and a 2D
// panel. The 3D part is about where the uncertainty is; the panel is about how
// bad it is and why.

#include <stdbool.h>

#include "raylib.h"
#include "map_session.h"
#include "quality.h"
#include "theme.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool show_gps_ring;      // h_acc circle on the ground
    bool show_covariance;    // LOCAL_POSITION_NED_COV uncertainty ellipsoid
    bool show_separation;    // closest-pair line between vehicles
    bool show_panel;         // the 2D numbers
} quality_overlay_opts_t;

void quality_overlay_defaults(quality_overlay_opts_t *o);

// World-space overlays. Call inside BeginMode3D.
void quality_overlay_draw_3d(const map_session_t *ms, const quality_overlay_opts_t *o,
                             int focus_vehicle, const theme_t *theme);

// The panel. Returns the height consumed so callers can stack panels.
int quality_overlay_draw_panel(const map_session_t *ms, const quality_overlay_opts_t *o,
                               int focus_vehicle, int x, int y, int width,
                               Font font, const theme_t *theme);

#ifdef __cplusplus
}
#endif

#endif
