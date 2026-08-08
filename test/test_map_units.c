// Focused checks for map behaviours that no scored fixture can reach.
//
// The fixtures score a finished map against known geometry, which is the right
// shape for anything that ends up as cells. It is the wrong shape for a rule
// about which of two origin messages wins, or what a negative field means --
// those either never produce a ray at all, or produce one whose wrongness is
// swamped by forty correct ones. Each case here was a real defect found in
// review; each asserts the specific thing that was broken.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fleet_frame.h"
#include "octomap.h"
#include "ray_transform.h"
#include "skynet_manifest.h"
#include "timebase.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG (M_PI / 180.0)

static int failures = 0;

static void check(bool ok, const char *what) {
    printf("  %-52s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) failures++;
}

// --------------------------------------------------------------- origins

// Hawkeye attaching mid-flight sees GLOBAL_POSITION_INT before the
// authoritative GPS_GLOBAL_ORIGIN. If the worse source wins, every later
// LOCAL_POSITION_NED resolves against an origin displaced by however far the
// vehicle had already flown.
static void test_origin_priority(void) {
    printf("origin priority\n");

    fleet_frame_t ff;
    fleet_frame_init(&ff, FLEET_ORIGIN_FIRST_SEEN);

    // 400 m north of the true origin: the vehicle's position when we attached.
    fleet_frame_note_origin(&ff, 0, FLEET_ORIGIN_SRC_FIRST_FIX,
                            47.401340, 8.545594, 489.4);
    check(ff.vehicle_source[0] == FLEET_ORIGIN_SRC_FIRST_FIX,
          "first fix is accepted when nothing better is known");

    const bool moved = fleet_frame_note_origin(&ff, 0, FLEET_ORIGIN_SRC_GPS_ORIGIN,
                                               47.397742, 8.545594, 489.4);
    check(ff.vehicle_source[0] == FLEET_ORIGIN_SRC_GPS_ORIGIN,
          "GPS_GLOBAL_ORIGIN replaces the earlier first fix");
    check(moved, "the session origin follows the upgrade");

    // ...and does not slide back when a later first fix arrives.
    fleet_frame_note_origin(&ff, 0, FLEET_ORIGIN_SRC_FIRST_FIX,
                            47.410000, 8.545594, 489.4);
    check(ff.vehicle_source[0] == FLEET_ORIGIN_SRC_GPS_ORIGIN,
          "a later first fix does not displace the GPS origin");

    // The damage the inversion caused, measured: a local NED of all zeros must
    // resolve to the session origin, not to 400 m north of it.
    fleet_frame_freeze(&ff);
    double enu[3] = { 0, 0, 0 };
    const double ned[3] = { 0, 0, 0 };
    check(fleet_frame_local_ned_to_enu(&ff, 0, ned, enu), "local NED resolves");
    const double err = sqrt(enu[0] * enu[0] + enu[1] * enu[1]);
    check(err < 1.0, "vehicle origin lands within a metre of the GPS origin");
}

// --------------------------------------------------------------- TIMESYNC

// In a TIMESYNC reply ts1 echoes the stamp we sent and tc1 carries the
// vehicle's clock. Crossing them subtracts a boot-relative clock from a wall
// clock, and because TIMESYNC outranks tlog arrival it then overwrites a
// correct alignment with one decades out.
static void test_timesync_roles(void) {
    printf("TIMESYNC round trip\n");

    timebase_session_t s;
    timebase_t tb;
    timebase_session_init(&s);
    timebase_init(&tb);

    const int64_t viewer_now = 1750000000000000000LL;   // viewer wall clock
    const int64_t boot_ns    = 120000000000LL;          // vehicle 120 s up

    // Establish the session epoch from arrival first, the way a live link does.
    timebase_observe_arrival(&s, &tb, boot_ns, viewer_now);

    // The reply: ts1 is our own send stamp coming back, tc1 is the vehicle's
    // clock at the moment it answered. 4 ms of round trip.
    const int64_t sent_at = viewer_now;
    const int64_t rtt     = 4000000LL;
    const int64_t local   = sent_at + rtt;
    timebase_observe_timesync(&s, &tb, sent_at, boot_ns + rtt / 2, local);

    check(tb.provenance == TIME_PROV_TIMESYNC, "TIMESYNC is adopted");
    check(tb.rtt_ns == rtt, "round trip is the viewer-side difference");

    // A vehicle stamp of boot_ns must land at the session instant the viewer
    // saw it, give or take half a round trip.
    const int64_t session_ns = timebase_to_session(&tb, boot_ns + rtt / 2);
    const int64_t expect_ns  = (sent_at + rtt / 2) - s.epoch_unix_ns;
    const int64_t skew = llabs(session_ns - expect_ns);
    printf("    session skew %lld ns\n", (long long)skew);
    check(skew < 10000000LL, "session time is within 10 ms of the truth");

    // Garbage in: a reply whose echo is in the future, or a round trip of a
    // minute, is not a time source.
    timebase_t bad;
    timebase_init(&bad);
    check(!timebase_observe_timesync(&s, &bad, local + 1000000LL, boot_ns, local),
          "a negative round trip is rejected");
    check(!timebase_observe_timesync(&s, &bad, local - 90000000000LL, boot_ns, local),
          "a 90-second round trip is rejected");
}

// ---------------------------------------------------------------- mounts

static void level_attitude(float q[4]) { q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f; }

static void test_orientation_table(void) {
    printf("sensor orientation table\n");

    ray_obs_t o;
    memset(&o, 0, sizeof(o));
    o.distance_m = 10.0f;
    o.max_distance_m = 40.0f;
    o.min_distance_m = 0.2f;
    o.covariance_cm2 = 1;
    o.signal_quality = 100;
    level_attitude(o.att_ned_body);

    om_ray_t ray;

    // 39 (PITCH_315) aims 45 degrees down; 40 (ROLL_90_PITCH_315) shares that
    // boresight. Before entry 40 existed it fell off the end of the table and
    // came out as a phantom obstacle 10 m dead ahead at flight level.
    // Never put a call inside assert(): NDEBUG compiles the whole expression
    // away, so in a release build the ray would never be built and everything
    // after this would compare uninitialised stack.
    o.orientation = 39;
    if (!rt_build_ray(&o, &ray)) {
        check(false, "orientation 39 builds a ray");
        return;
    }
    const float down39 = ray.endpoint[2];

    o.orientation = 40;
    check(rt_build_ray(&o, &ray), "orientation 40 builds a ray");
    check(fabsf(ray.endpoint[2] - down39) < 0.01f,
          "orientation 40 aims 45 degrees down, not straight ahead");
    check(ray.endpoint[2] < -6.0f, "...and the endpoint really is below the vehicle");

    // Past the end of the enum there is no defensible direction, so there is no
    // ray. Inventing body +X carves free space through wherever the sensor was
    // actually pointing, silently.
    o.orientation = 41;
    check(!rt_build_ray(&o, &ray), "an unknown orientation is rejected, not guessed");
    o.orientation = 100;   // MAV_SENSOR_ROTATION_CUSTOM without a quaternion
    check(!rt_build_ray(&o, &ray), "CUSTOM without a quaternion is rejected");

    // CUSTOM *with* a quaternion is the supported case.
    o.orientation = 100;
    o.have_quaternion = true;
    rt_quat_from_euler(0.0f, 0.0f, (float)(90.0 * DEG), o.sensor_q);
    check(rt_build_ray(&o, &ray), "CUSTOM with a quaternion builds a ray");
    check(ray.endpoint[0] > 9.0f, "...and points east, as the quaternion says");
}

// ------------------------------------------------------ obstacle fan sign

// increment_f replaces the integer increment whenever it is non-zero. A
// negative value means the fan sweeps counter-clockwise; dropping the sign
// mirrors every obstacle onto the wrong side of the vehicle.
static void test_negative_increment(void) {
    printf("OBSTACLE_DISTANCE increment sign\n");

    obstacle_obs_t o;
    memset(&o, 0, sizeof(o));
    o.sector_count = 8;
    o.frame = RT_FRAME_BODY_FRD;
    o.min_distance_cm = 20;
    o.max_distance_cm = 4000;
    o.angle_offset_deg = 0.0f;
    for (int i = 0; i < 8; i++) o.distances_cm[i] = UINT16_MAX;
    o.distances_cm[1] = 1000;    // one return, one sector off the nose
    level_attitude(o.att_ned_body);

    om_ray_t rays[OBSTACLE_DISTANCE_SECTORS];

    o.increment_deg = 45.0f;
    int n = rt_expand_obstacle_distance(&o, rays, OBSTACLE_DISTANCE_SECTORS);
    check(n == 1, "one populated sector yields one ray");
    const double cw_e = rays[0].endpoint[0], cw_n = rays[0].endpoint[1];

    o.increment_deg = -45.0f;
    n = rt_expand_obstacle_distance(&o, rays, OBSTACLE_DISTANCE_SECTORS);
    check(n == 1, "a negative increment still yields a ray");
    const double ccw_e = rays[0].endpoint[0], ccw_n = rays[0].endpoint[1];

    printf("    +45 -> E %+.2f N %+.2f    -45 -> E %+.2f N %+.2f\n",
           cw_e, cw_n, ccw_e, ccw_n);
    check(fabs(cw_n - ccw_n) < 0.01, "both land the same distance up-track");
    check(cw_e * ccw_e < 0.0, "the sign of the increment mirrors the bearing");
    check(fabs(cw_e + ccw_e) < 0.01, "...symmetrically about the nose");
}

// ------------------------------------------------------------ pool budget

// pool_reserve used to refuse any growth that overshot byte_cap and, changing
// no state, refuse identically forever after -- so the map ran at half its
// advertised budget while subdivisions were turned away by the hundred
// thousand.
static void test_pool_uses_full_budget(void) {
    printf("node pool budget\n");

    const size_t cap_bytes = 16u << 20;
    octomap_t m;
    octomap_config_t cfg;
    octomap_config_defaults(&cfg);
    cfg.byte_cap = cap_bytes;
    if (octomap_init(&m, &cfg) != 0) {
        check(false, "the map initialises with a 16 MiB cap");
        return;
    }

    // Enough scattered rays to exhaust the budget several times over.
    uint32_t seed = 12345u;
    for (int i = 0; i < 120000; i++) {
        seed = seed * 1664525u + 1013904223u;
        const double a = (double)(seed >> 8) / (double)(1u << 24) * 2.0 * M_PI;
        seed = seed * 1664525u + 1013904223u;
        const double b = (double)(seed >> 8) / (double)(1u << 24) * M_PI - M_PI * 0.5;
        om_ray_t r;
        memset(&r, 0, sizeof(r));
        r.origin[0] = 0.0f; r.origin[1] = 0.0f; r.origin[2] = 20.0f;
        r.endpoint[0] = (float)(cos(b) * cos(a) * 60.0);
        r.endpoint[1] = (float)(cos(b) * sin(a) * 60.0);
        r.endpoint[2] = (float)(20.0 + sin(b) * 60.0);
        r.hit = 1;
        r.weight = 1.0f;
        octomap_insert_ray(&m, &r);
    }

    const size_t used = octomap_bytes(&m);
    printf("    %.2f MiB of a %.0f MiB cap, %llu refusals\n",
           (double)used / 1048576.0, (double)cap_bytes / 1048576.0,
           (unsigned long long)m.stats.alloc_refusals);
    check(used <= cap_bytes, "the pool stays inside its byte cap");
    check(used > cap_bytes * 3 / 4,
          "the pool reaches most of its cap rather than stalling at half");

    octomap_free(&m);
}

// ------------------------------------------------------- manifest nesting

static void test_manifest_depth(void) {
    printf("run manifest nesting\n");

    char path[512];
    snprintf(path, sizeof(path), "deep_manifest.json");
    FILE *f = fopen(path, "wb");
    if (!f) { printf("  cannot write %s -- skipped\n", path); return; }
    fputs("{\"logs\":[{\"path\":\"a.tlog\",\"x\":", f);
    const int depth = 100000;
    for (int i = 0; i < depth; i++) fputc('[', f);
    for (int i = 0; i < depth; i++) fputc(']', f);
    fputs("}]}", f);
    fclose(f);

    skynet_manifest_t m;
    char err[256] = {0};
    const int rc = skynet_manifest_load(&m, path, err, sizeof(err));
    printf("    rc=%d err=\"%s\"\n", rc, err);
    check(rc != 0, "a 100000-deep manifest is refused rather than crashing");
    check(err[0] != '\0', "...with a message naming the line");
    remove(path);

    // The shallow case still loads, so the guard has not simply banned nesting.
    f = fopen(path, "wb");
    if (f) {
        fputs("{\"run_id\":\"r1\",\"origin\":{\"lat\":47.4,\"lon\":8.5},"
              "\"logs\":[{\"path\":\"a.tlog\",\"kind\":\"tlog\",\"sysid\":1}]}", f);
        fclose(f);
        const int ok = skynet_manifest_load(&m, path, err, sizeof(err));
        check(ok == 0 && m.entry_count == 1, "an ordinary manifest still loads");
        remove(path);
    }
}

int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);

    test_origin_priority();
    test_timesync_roles();
    test_orientation_table();
    test_negative_increment();
    test_pool_uses_full_budget();
    test_manifest_depth();

    printf("\n%s (%d failure%s)\n", failures ? "FAIL" : "PASS",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
