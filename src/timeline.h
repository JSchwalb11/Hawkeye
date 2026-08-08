#ifndef TIMELINE_H
#define TIMELINE_H

// The time-indexed core. Live is replay with the playhead pinned to now.
//
// There is one playhead. Live pins it to the head of the data; scrubbing
// unpins it; the live button re-pins. That is the whole live/replay
// unification -- one code path, and the live view gains rewind for free, which
// is exactly what you want the moment a vehicle does something odd.
//
// Three things are stored against session time:
//
//  * per-vehicle decoded state, in a byte-budgeted ring with a keyframe index
//    so seeking is a binary search rather than a replay from zero;
//  * every ray that went into the map, so scrubbing backwards can rebuild the
//    map as it stood -- otherwise scrubbing back shows obstacles that had not
//    been discovered yet;
//  * event marks (STATUSTEXT, mode changes, command acks, mission items, and
//    the map's own ray drops), which are the flight's narrative rather than
//    just its trajectory.
//
// Ring sizes are byte budgets, not durations. A 16-vehicle firehose and a
// single hovering quad should occupy the same memory, not the same wall time.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "octomap.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TL_MAX_VEHICLES 256
#define TL_EVENT_TEXT   50

// --- Vehicle state samples ---------------------------------------------

typedef struct {
    int64_t t_ns;          // session time
    double  enu[3];        // session ENU metres
    float   q[4];          // attitude, NED<-body
    float   vel_ned[3];
    uint8_t nav_state;     // flight mode as reported (0xFF = unknown)
    uint8_t flags;
} tl_sample_t;

#define TL_SAMPLE_POS_VALID  0x01
#define TL_SAMPLE_ATT_VALID  0x02

typedef struct {
    tl_sample_t *ring;
    uint32_t     cap;
    uint32_t     count;      // live entries, <= cap
    uint32_t     head;       // next write slot
    uint64_t     total;      // lifetime samples, for the keyframe stride
    bool         active;
} tl_track_t;

// --- Event marks --------------------------------------------------------

typedef enum {
    TL_EVENT_STATUSTEXT = 0,
    TL_EVENT_MODE_CHANGE,
    TL_EVENT_COMMAND_ACK,
    TL_EVENT_MISSION_ITEM,
    TL_EVENT_SYSTEM_EVENT,     // EVENT / CURRENT_EVENT_SEQUENCE
    TL_EVENT_MAP_DROP,         // rays shed because the map fell behind
    TL_EVENT_TIME_REALIGN,     // a source's time provenance changed
    TL_EVENT_KIND_COUNT
} tl_event_kind_t;

const char *tl_event_kind_name(tl_event_kind_t kind);

typedef struct {
    int64_t  t_ns;
    uint8_t  vehicle_id;
    uint8_t  kind;
    uint8_t  severity;     // MAV_SEVERITY where meaningful, else 6 (info)
    uint32_t code;         // command id, mission seq, event id, dropped count
    char     text[TL_EVENT_TEXT];
} tl_event_t;

// --- Ray log ------------------------------------------------------------

// Compact enough that a byte budget buys a useful amount of history. Float
// precision is ~1e-7 relative, far below the 0.25 m leaf size at any range the
// root cube covers.
#define TL_RAY_SKIPPED 0x01   // shed by the ingest queue; the map never saw it

typedef struct {
    int64_t  t_ns;
    float    origin[3];
    float    endpoint[3];
    uint8_t  vehicle_id;
    uint8_t  hit;
    uint8_t  weight_q;     // weight * 255
    uint8_t  cone_cm;      // cone radius in centimetres, saturating
    uint8_t  flags;
} tl_ray_t;

// --- Map keyframes ------------------------------------------------------

typedef struct {
    int64_t  t_ns;
    uint64_t ray_index;    // lifetime ray index this snapshot was taken at
    void    *blob;
    size_t   len;
} tl_keyframe_t;

// --- Playhead -----------------------------------------------------------

typedef struct {
    int64_t t_ns;
    bool    pinned_to_head;   // true == live
    bool    paused;
    float   speed;            // playback multiplier when unpinned
    bool    step_request;     // single-frame step, consumed by the advance call
} tl_playhead_t;

// --- Configuration ------------------------------------------------------

typedef struct {
    size_t   track_bytes_per_vehicle;  // 0 -> 4 MiB
    size_t   ray_log_bytes;            // 0 -> 64 MiB
    size_t   keyframe_bytes;           // 0 -> 96 MiB
    uint32_t event_capacity;           // 0 -> 4096
    float    keyframe_interval_s;      // 0 -> 20
    int      max_vehicles;             // 0 -> 16
} timeline_config_t;

typedef struct {
    tl_track_t  tracks[TL_MAX_VEHICLES];
    int         max_vehicles;
    size_t      track_bytes;

    tl_event_t *events;
    uint32_t    event_cap, event_count, event_head;

    tl_ray_t   *rays;
    uint32_t    ray_cap, ray_count, ray_head;
    uint64_t    ray_total;        // lifetime count; ring holds the newest ray_count
    uint64_t    ray_oldest;       // lifetime index of the oldest ray still held

    tl_keyframe_t *keyframes;
    uint32_t    kf_cap, kf_count;
    size_t      kf_bytes, kf_bytes_cap;
    float       kf_interval_s;
    int64_t     kf_last_ns;

    tl_playhead_t playhead;
    int64_t     head_ns;          // newest session timestamp seen
    int64_t     start_ns;         // oldest session timestamp seen
    bool        have_span;

    // Reconstruction cursor: the map currently reflects rays [.., map_cursor).
    uint64_t    map_cursor;
    int64_t     map_state_ns;
    bool        map_valid;
} timeline_t;

int  timeline_init(timeline_t *tl, const timeline_config_t *cfg);
void timeline_free(timeline_t *tl);
void timeline_clear(timeline_t *tl);

// --- Ingestion ----------------------------------------------------------

void timeline_add_sample(timeline_t *tl, uint8_t vehicle_id, const tl_sample_t *s);
void timeline_add_event(timeline_t *tl, const tl_event_t *e);
void timeline_note_drop(timeline_t *tl, int64_t t_ns, uint8_t vehicle_id, uint32_t dropped);

// Record a ray in the history. Call for every ray handed to the ingest queue;
// the returned lifetime index is what the queue reports progress in. Scrubbing
// rebuilds the map from this log, so it has to describe exactly what the map
// was built from -- no more and no less.
uint64_t timeline_add_ray(timeline_t *tl, const om_ray_t *ray, int64_t t_ns);

// Mark a recorded ray as never having reached the map. Replay skips it, so a
// reconstruction cannot end up denser than what the operator actually saw.
void timeline_mark_ray_skipped(timeline_t *tl, uint64_t seq);

// --- Query --------------------------------------------------------------

// Nearest sample at or before t_ns. Returns false when the track has nothing.
bool timeline_sample_at(const timeline_t *tl, uint8_t vehicle_id, int64_t t_ns,
                        tl_sample_t *out);

// Linearly interpolated pose at t_ns; falls back to the nearest sample at the
// ends of the track.
bool timeline_pose_at(const timeline_t *tl, uint8_t vehicle_id, int64_t t_ns,
                      tl_sample_t *out);

// Events in [t0, t1], newest first, up to max_out. Returns the count written.
int timeline_events_between(const timeline_t *tl, int64_t t0, int64_t t1,
                            const tl_event_t **out, int max_out);

int64_t timeline_head_ns(const timeline_t *tl);
int64_t timeline_start_ns(const timeline_t *tl);

// Earliest time the map can be faithfully reconstructed to. Older than this the
// ray history has been evicted, and the UI should say so rather than pretend.
int64_t timeline_map_history_start_ns(const timeline_t *tl);

// --- Playhead -----------------------------------------------------------

void timeline_pin_live(timeline_t *tl);
void timeline_unpin(timeline_t *tl);
void timeline_set_playhead(timeline_t *tl, int64_t t_ns);

// Advance the playhead by dt seconds of wall clock. When pinned, the playhead
// tracks the head; when unpinned it runs at `speed`, or single-steps.
void timeline_advance(timeline_t *tl, float dt_s);

// --- Map reconstruction -------------------------------------------------

// Bring `map` into agreement with the playhead. Forward moves replay the rays
// in between; backward moves restore the nearest keyframe first. Returns the
// number of rays replayed.
uint32_t timeline_sync_map(timeline_t *tl, octomap_t *map);

// Take a map keyframe if the interval has elapsed. Cheap no-op otherwise.
void timeline_maybe_keyframe(timeline_t *tl, const octomap_t *map, int64_t t_ns);

// Force a keyframe now, regardless of interval.
bool timeline_take_keyframe(timeline_t *tl, const octomap_t *map, int64_t t_ns);

size_t timeline_bytes(const timeline_t *tl);

#ifdef __cplusplus
}
#endif

#endif
