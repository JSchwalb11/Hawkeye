#include "quality_overlay.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "fleet_frame.h"
#include "raymath.h"

#define QO_LINE_H 15
#define QO_PAD    8

void quality_overlay_defaults(quality_overlay_opts_t *o) {
    if (!o) return;
    o->show_gps_ring = true;
    o->show_covariance = true;
    o->show_separation = true;
    o->show_panel = true;
}

static Color severity_colour(float ratio) {
    // Ratios are normalized so 1.0 is the rejection threshold; colour follows
    // the same scale whichever autopilot supplied the number.
    if (ratio < 0.5f) return (Color){ 123, 228, 149, 220 };
    if (ratio < 1.0f) return (Color){ 255, 209, 102, 230 };
    return (Color){ 255,  99, 132, 240 };
}

static bool vehicle_world(const map_session_t *ms, int slot, Vector3 *out) {
    if (slot < 0 || slot >= MS_MAX_VEHICLES) return false;
    const map_vehicle_t *v = &ms->veh[slot];
    if (!v->present || !v->pos_valid) return false;
    float w[3];
    fleet_enu_to_world(v->enu, w);
    out->x = w[0]; out->y = w[1]; out->z = w[2];
    return true;
}

// ---------------------------------------------------------------- 3D

static void draw_ring(Vector3 centre, float radius, Color c, int segments) {
    if (radius <= 0.0f || !isfinite(radius)) return;
    Vector3 prev = { centre.x + radius, centre.y, centre.z };
    for (int i = 1; i <= segments; i++) {
        const float a = (float)i / segments * 2.0f * PI;
        const Vector3 p = { centre.x + cosf(a) * radius, centre.y, centre.z + sinf(a) * radius };
        DrawLine3D(prev, p, c);
        prev = p;
    }
}

// The 1-sigma horizontal ellipse from a real covariance, oriented by its own
// principal axis. A circle would be a different claim than the estimator made.
static void draw_covariance_ellipse(Vector3 centre, float semi_major, float semi_minor,
                                    float angle_rad, Color c) {
    if (!(semi_major > 0.0f) || !isfinite(semi_major)) return;
    const int segments = 48;
    Vector3 prev = { 0 };
    for (int i = 0; i <= segments; i++) {
        const float t = (float)i / segments * 2.0f * PI;
        const float ex = cosf(t) * semi_major;
        const float ey = sinf(t) * semi_minor;
        // Covariance is in NED; the world is X east, Z south.
        const float n = ex * cosf(angle_rad) - ey * sinf(angle_rad);
        const float e = ex * sinf(angle_rad) + ey * cosf(angle_rad);
        const Vector3 p = { centre.x + e, centre.y, centre.z - n };
        if (i > 0) DrawLine3D(prev, p, c);
        prev = p;
    }
}

void quality_overlay_draw_3d(const map_session_t *ms, const quality_overlay_opts_t *o,
                             int focus_vehicle, const theme_t *theme) {
    (void)theme;
    if (!ms || !o) return;

    for (int i = 0; i < MS_MAX_VEHICLES; i++) {
        const map_vehicle_t *v = &ms->veh[i];
        if (!v->present || !v->pos_valid) continue;
        Vector3 p;
        if (!vehicle_world(ms, i, &p)) continue;
        const Vector3 ground = { p.x, 0.05f, p.z };

        if (o->show_gps_ring && v->quality.h_acc_valid) {
            // h_acc is millimetres of horizontal accuracy. eph would be HDOP
            // times 100 -- dimensionless, and a ring drawn at it means nothing.
            const Color c = (v->quality.fix_type >= 5)
                ? (Color){ 123, 228, 149, 150 } : (Color){ 255, 209, 102, 150 };
            draw_ring(ground, v->quality.h_acc_m, c, 48);
        }

        if (o->show_covariance) {
            float a = 0.0f, b = 0.0f, ang = 0.0f;
            if (quality_horizontal_sigma(&v->quality, &a, &b, &ang))
                draw_covariance_ellipse(p, a, b, ang, (Color){ 199, 146, 234, 190 });
        }
    }

    if (o->show_separation) {
        double enu[MS_MAX_VEHICLES][3];
        bool valid[MS_MAX_VEHICLES];
        int n = 0;
        for (int i = 0; i < MS_MAX_VEHICLES; i++) {
            valid[n] = ms->veh[i].present && ms->veh[i].pos_valid;
            memcpy(enu[n], ms->veh[i].enu, sizeof(enu[0]));
            n++;
        }
        separation_pair_t pair;
        if (quality_closest_pair((const double (*)[3])enu, valid, n, &pair)) {
            Vector3 a, b;
            if (vehicle_world(ms, pair.a, &a) && vehicle_world(ms, pair.b, &b)) {
                const Color c = (pair.distance_m < 10.0f)
                    ? (Color){ 255, 99, 132, 220 } : (Color){ 139, 152, 165, 130 };
                DrawLine3D(a, b, c);
            }
        }
    }
    (void)focus_vehicle;
}

// ---------------------------------------------------------------- panel

static void row(Font f, int x, int *y, const char *label, const char *value, Color vc) {
    DrawTextEx(f, label, (Vector2){ (float)x, (float)*y }, 12, 1.0f,
               (Color){ 139, 152, 165, 235 });
    DrawTextEx(f, value, (Vector2){ (float)(x + 118), (float)*y }, 12, 1.0f, vc);
    *y += QO_LINE_H;
}

int quality_overlay_draw_panel(const map_session_t *ms, const quality_overlay_opts_t *o,
                               int focus_vehicle, int x, int y, int width,
                               Font font, const theme_t *theme) {
    (void)theme;
    if (!ms || !o || !o->show_panel) return 0;
    if (focus_vehicle < 0 || focus_vehicle >= MS_MAX_VEHICLES) return 0;
    const map_vehicle_t *v = &ms->veh[focus_vehicle];
    if (!v->present) return 0;
    const vehicle_quality_t *q = &v->quality;

    const int height = 232;
    DrawRectangle(x, y, width, height, (Color){ 14, 17, 22, 215 });
    DrawRectangleLines(x, y, width, height, (Color){ 42, 51, 65, 255 });

    int ty = y + QO_PAD;
    char buf[96];
    snprintf(buf, sizeof(buf), "QUALITY  V%d", focus_vehicle);
    DrawTextEx(font, buf, (Vector2){ (float)(x + QO_PAD), (float)ty }, 13, 1.0f,
               (Color){ 230, 237, 243, 255 });
    ty += 20;

    const int lx = x + QO_PAD;

    // GPS. h_acc is the metric ring; HDOP/VDOP are numbers, not distances.
    if (q->gps_valid) {
        static const char *const fix[] = { "NO GPS", "NO FIX", "2D", "3D", "DGPS",
                                           "RTK FLOAT", "RTK FIXED" };
        const char *fx = (q->fix_type < 7) ? fix[q->fix_type] : "?";
        snprintf(buf, sizeof(buf), "%s  %u sat", fx, q->satellites_visible);
        row(font, lx, &ty, "FIX", buf,
            q->fix_type >= 3 ? (Color){ 123, 228, 149, 255 } : (Color){ 255, 99, 132, 255 });

        if (q->h_acc_valid) snprintf(buf, sizeof(buf), "%.2f m / %.2f m", q->h_acc_m, q->v_acc_m);
        else snprintf(buf, sizeof(buf), "not reported");
        row(font, lx, &ty, "H/V ACC", buf, (Color){ 230, 237, 243, 255 });

        if (isfinite(q->hdop)) snprintf(buf, sizeof(buf), "%.2f / %.2f", q->hdop, q->vdop);
        else snprintf(buf, sizeof(buf), "--");
        row(font, lx, &ty, "HDOP/VDOP", buf, (Color){ 230, 237, 243, 255 });
    } else {
        row(font, lx, &ty, "FIX", "no GPS_RAW_INT", (Color){ 139, 152, 165, 255 });
    }

    // Estimator health, normalized so PX4 ratios and ArduPilot variances share
    // one scale.
    if (q->est_px4_valid || q->est_ardu_valid) {
        const float health = quality_estimator_health(q);
        snprintf(buf, sizeof(buf), "%.2f  (1.0 = reject)", health);
        row(font, lx, &ty, q->est_px4_valid ? "EKF INNOV" : "EKF VAR", buf,
            severity_colour(health));

        const int bar_w = width - 2 * QO_PAD;
        const float fill = fminf(1.0f, health);
        DrawRectangle(lx, ty, bar_w, 5, (Color){ 30, 36, 46, 255 });
        DrawRectangle(lx, ty, (int)(bar_w * fill), 5, severity_colour(health));
        ty += 12;
    }

    if (q->cov_valid) {
        float a = 0, b = 0, ang = 0;
        if (quality_horizontal_sigma(q, &a, &b, &ang)) {
            snprintf(buf, sizeof(buf), "%.2f x %.2f m", a, b);
            row(font, lx, &ty, "1-SIGMA", buf, (Color){ 199, 146, 234, 255 });
        }
    }

    // Link: round-trip and sequence-gap loss.
    if (q->rtt_ms > 0.0f || q->seq_received > 0) {
        snprintf(buf, sizeof(buf), "%.1f ms  %.2f%% loss", q->rtt_ms, q->loss_pct);
        row(font, lx, &ty, "LINK", buf,
            q->loss_pct > 2.0f ? (Color){ 255, 209, 102, 255 } : (Color){ 230, 237, 243, 255 });
    }

    if (q->wind_valid) {
        const float speed = sqrtf(q->wind_ned[0] * q->wind_ned[0]
                                + q->wind_ned[1] * q->wind_ned[1]);
        float dir = atan2f(q->wind_ned[1], q->wind_ned[0]) * RAD2DEG;
        if (dir < 0.0f) dir += 360.0f;
        snprintf(buf, sizeof(buf), "%.1f m/s  %.0f deg", speed, dir);
        row(font, lx, &ty, "WIND", buf, (Color){ 230, 237, 243, 255 });
    }

    if (q->vibe_valid) {
        const uint32_t clip = q->clipping[0] + q->clipping[1] + q->clipping[2];
        snprintf(buf, sizeof(buf), "%.1f/%.1f/%.1f  clip %u",
                 q->vibration[0], q->vibration[1], q->vibration[2], clip);
        row(font, lx, &ty, "VIBE", buf,
            clip > 0 ? (Color){ 255, 99, 132, 255 } : (Color){ 230, 237, 243, 255 });
    }

    if (q->clearance_valid || q->terrain_valid) {
        if (q->clearance_valid) snprintf(buf, sizeof(buf), "%.2f m AGL", q->bottom_clearance);
        else snprintf(buf, sizeof(buf), "%.1f m terrain", q->current_height);
        row(font, lx, &ty, "CLEARANCE", buf, (Color){ 230, 237, 243, 255 });
    }

    // Conflict: prefer what the vehicle reported, fall back to the derived
    // separation matrix.
    if (q->collision_reported) {
        snprintf(buf, sizeof(buf), "COLLISION lvl %u  %.1f s",
                 q->collision_threat_level, q->collision_time_to_min_delta);
        row(font, lx, &ty, "CONFLICT", buf, (Color){ 255, 99, 132, 255 });
    } else {
        double enu[MS_MAX_VEHICLES][3];
        bool valid[MS_MAX_VEHICLES];
        int n = 0;
        for (int i = 0; i < MS_MAX_VEHICLES; i++) {
            valid[n] = ms->veh[i].present && ms->veh[i].pos_valid;
            memcpy(enu[n], ms->veh[i].enu, sizeof(enu[0]));
            n++;
        }
        separation_pair_t pair;
        if (quality_closest_pair((const double (*)[3])enu, valid, n, &pair)) {
            snprintf(buf, sizeof(buf), "V%d-V%d  %.1f m", pair.a, pair.b, pair.distance_m);
            row(font, lx, &ty, "CLOSEST", buf,
                pair.distance_m < 10.0f ? (Color){ 255, 99, 132, 255 }
                                        : (Color){ 230, 237, 243, 255 });
        }
    }

    return height;
}
