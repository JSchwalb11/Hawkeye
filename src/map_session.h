#ifndef MAP_SESSION_H
#define MAP_SESSION_H

// The single seam every data source pours into.
//
// A source's only job is to produce (session time, vehicle, decoded frame). It
// hands that to this object, which owns the shared frame, the time base, the
// map, the ray queue and the timeline. Nothing here knows or cares whether the
// bytes came from a live radio, a tlog, a ULog, a DataFlash log, or a test
// fixture -- if DISTANCE_SENSOR arrives, it is truth.

#include <stdbool.h>
#include <stdint.h>

#include "fleet_frame.h"
#include "map_ingest.h"
#include "octomap.h"
#include "quality.h"
#include "ray_transform.h"
#include "timebase.h"
#include "timeline.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MS_MAX_VEHICLES TL_MAX_VEHICLES

typedef struct {
    bool     present;
    uint8_t  sysid;
    uint8_t  compid;
    uint8_t  mav_type;
    uint8_t  autopilot;
    uint32_t custom_mode;
    uint8_t  base_mode;
    bool     have_mode;

    timebase_t tb;
    int64_t  src_now_ns;      // this vehicle's own clock, latest value seen
    bool     have_src_time;

    // Latest decoded pose, in the session frame.
    double   enu[3];
    bool     pos_valid;
    int64_t  pos_t_ns;
    float    q_ned_body[4];
    bool     att_valid;
    int64_t  att_t_ns;

    double   lat_deg, lon_deg, alt_m;
    bool     global_valid;

    vehicle_quality_t quality;

    uint8_t  last_seq;
    bool     have_seq;

    uint64_t rays_contributed;
    uint64_t rays_rejected;
    bool     muted;           // excluded from the map, kept on the timeline
} map_vehicle_t;

typedef struct {
    size_t   map_byte_cap;      // 0 -> octomap default
    double   root_size_m;
    int      max_depth;
    int      coarse_depth;
    double   skip_near_m;
    uint32_t queue_capacity;
    uint32_t budget_per_drain;
    size_t   ray_log_bytes;
    size_t   keyframe_bytes;
    float    keyframe_interval_s;
    int      max_vehicles;
    fleet_origin_policy_t origin_policy;
} map_session_config_t;

typedef struct {
    octomap_t     map;
    map_ingest_t  ingest;
    timeline_t    timeline;
    fleet_frame_t frame;
    timebase_session_t clock;

    map_vehicle_t veh[MS_MAX_VEHICLES];
    int           vehicle_count;
    int           focus_vehicle;      // -1 = none
    int           solo_vehicle;       // -1 = none

    uint64_t      last_reported_drops;
    bool          enabled;            // false suspends map building entirely
} map_session_t;

void map_session_config_defaults(map_session_config_t *cfg);
int  map_session_init(map_session_t *ms, const map_session_config_t *cfg);
void map_session_free(map_session_t *ms);

// Find or allocate the slot for a system id.
int  map_session_slot_for_sysid(map_session_t *ms, uint8_t sysid);

// Bind a slot to a system id up front (one source per vehicle, as `-n` does).
void map_session_bind_slot(map_session_t *ms, int slot, uint8_t sysid);

// --- Generic feeds, source-agnostic ------------------------------------

void map_session_feed_origin(map_session_t *ms, int slot, uint8_t src,
                             double lat_deg, double lon_deg, double alt_m);

void map_session_feed_global(map_session_t *ms, int slot, int64_t t_ns,
                             double lat_deg, double lon_deg, double alt_m,
                             const float vel_ned[3]);

void map_session_feed_local_ned(map_session_t *ms, int slot, int64_t t_ns,
                                const double ned[3], const float vel_ned[3]);

void map_session_feed_attitude(map_session_t *ms, int slot, int64_t t_ns,
                               const float q_ned_body[4]);

// A ranging observation. `obs` needs only the sensor fields filled in: the
// origin and attitude are resolved from the timeline at the observation's own
// timestamp, which is what stops a fresh range paired with a stale attitude
// from smearing a flat wall into a curve.
void map_session_feed_distance(map_session_t *ms, int slot, int64_t t_ns,
                               const ray_obs_t *obs);

void map_session_feed_obstacle(map_session_t *ms, int slot, int64_t t_ns,
                               const obstacle_obs_t *obs);

void map_session_feed_event(map_session_t *ms, int slot, int64_t t_ns,
                            tl_event_kind_t kind, uint8_t severity,
                            uint32_t code, const char *text);

// --- Per-frame work ----------------------------------------------------

// Drain the ray queue into the map, take keyframes, and keep the playhead in
// step. Call once per frame with the wall-clock delta.
void map_session_tick(map_session_t *ms, float dt_s);

// The same work, but with the playhead set from outside instead of advanced
// by dt_s. Replay uses this so the transport stays the single authority on
// "now" without the map growing a second, divergent code path.
void map_session_tick_at(map_session_t *ms, int64_t want_ns, float dt_s);

// Bring the map into agreement with the playhead after a scrub.
uint32_t map_session_resync(map_session_t *ms);

// --- Focus / mute ------------------------------------------------------

void map_session_set_focus(map_session_t *ms, int slot);
void map_session_set_mute(map_session_t *ms, int slot, bool muted);
void map_session_set_solo(map_session_t *ms, int slot);   // -1 clears
bool map_session_vehicle_contributes(const map_session_t *ms, int slot);

// Bitmask of the focused vehicle's contribution, for the render highlight.
uint32_t map_session_focus_mask(const map_session_t *ms);

// --- Reporting ---------------------------------------------------------

// Worst time provenance across contributing vehicles, and the spread between
// the best- and worst-aligned source. Both belong on screen.
time_provenance_t map_session_time_provenance(const map_session_t *ms);
int64_t map_session_time_spread_ns(const map_session_t *ms);

#ifdef __cplusplus
}
#endif

#endif
