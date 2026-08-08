#ifndef TIMEBASE_H
#define TIMEBASE_H

// One monotonic session timeline, in nanoseconds, that every data source maps
// into.
//
// Sources disagree about time. ULog is boot-relative. MAVLink `time_usec` is
// boot-relative on PX4 and mixed on ArduPilot. tlog carries wall-clock arrival.
// DataFlash has its own. A fleet replay in which two vehicles sit seconds apart
// is not a replay of anything, so each source declares an offset that maps its
// own stamps into session time, together with the provenance of that offset.
//
// Provenance is not decoration. An unlabelled alignment guess is a lie the
// viewer tells quietly, so the offset and how it was obtained are both carried
// out to the UI.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Ordered worst-to-best; a source only ever upgrades its provenance.
typedef enum {
    TIME_PROV_NONE = 0,        // nothing seen yet; offset is zero by default
    TIME_PROV_BOOT_ASSUMED,    // boot-relative, offset assumed zero
    TIME_PROV_BOOT_MANUAL,     // boot-relative, offset set by the operator
    TIME_PROV_TLOG_ARRIVAL,    // wall-clock arrival stamped by the recorder
    TIME_PROV_TIMESYNC,        // TIMESYNC round-trip against the viewer clock
    TIME_PROV_GPS_RAW,         // GPS_RAW_INT.time_usec (GPS epoch)
    TIME_PROV_GPS_UNIX,        // SYSTEM_TIME.time_unix_usec
    TIME_PROV_COUNT
} time_provenance_t;

// Short label for the UI. Never returns NULL.
const char *timebase_provenance_name(time_provenance_t p);

// Longer one-line explanation for tooltips / --help output.
const char *timebase_provenance_detail(time_provenance_t p);

typedef struct {
    int64_t           offset_ns;     // session_ns = source_ns + offset_ns
    time_provenance_t provenance;
    bool              locked;        // operator pinned the offset; observations stop upgrading it
    int64_t           first_source_ns;
    int64_t           last_source_ns;
    bool              have_first;

    // TIMESYNC accounting, surfaced by the link overlay.
    int64_t           rtt_ns;        // last measured round-trip
    int64_t           rtt_ns_min;
    uint32_t          timesync_count;
} timebase_t;

// The session-wide clock everything is expressed against.
typedef struct {
    int64_t epoch_unix_ns;   // session t=0 in UNIX nanoseconds (0 = not yet known)
    bool    epoch_known;
    int64_t horizon_ns;      // greatest session timestamp seen from any source
    bool    have_horizon;
} timebase_session_t;

void timebase_session_init(timebase_session_t *s);
void timebase_init(timebase_t *tb);

// Note a session timestamp so the session horizon (== "now" for live) tracks it.
void timebase_session_note(timebase_session_t *s, int64_t session_ns);

// --- Observations, best first. Each returns true if it changed the offset. ---

// SYSTEM_TIME: pairs a boot-relative stamp with a UNIX stamp from the vehicle.
bool timebase_observe_system_time(timebase_session_t *s, timebase_t *tb,
                                  uint64_t time_boot_ms, uint64_t time_unix_usec);

// GPS_RAW_INT.time_usec is GPS-epoch microseconds; converted to UNIX using the
// current leap-second count.
bool timebase_observe_gps_raw(timebase_session_t *s, timebase_t *tb,
                              uint64_t source_time_usec, uint64_t gps_time_usec);

// TIMESYNC round-trip. `ts1` is the remote stamp, `tc1` the local echo, and
// local_ns the viewer clock at receipt.
bool timebase_observe_timesync(timebase_session_t *s, timebase_t *tb,
                               int64_t ts1_ns, int64_t tc1_ns, int64_t local_ns);

// tlog arrival: the recorder's wall clock at the moment the frame landed.
bool timebase_observe_arrival(timebase_session_t *s, timebase_t *tb,
                              int64_t source_ns, int64_t arrival_unix_ns);

// Operator override. Always wins and locks the source until unlocked.
void timebase_set_manual(timebase_t *tb, int64_t offset_ns);
void timebase_unlock(timebase_t *tb);

// Map a source timestamp into session time.
int64_t timebase_to_session(const timebase_t *tb, int64_t source_ns);

// Map back, for sources that need to seek in their own units.
int64_t timebase_from_session(const timebase_t *tb, int64_t session_ns);

// Worst provenance across a fleet — what the UI should warn about.
time_provenance_t timebase_fleet_worst(const timebase_t *const *tbs, int count);

// Largest pairwise offset spread across the fleet, in nanoseconds. This is the
// number that says "your two vehicles are N ms apart".
int64_t timebase_fleet_spread_ns(const timebase_t *const *tbs, int count);

// GPS epoch (1980-01-06T00:00:00Z) expressed in UNIX seconds, and the leap
// second count in effect. Both exposed so tests can assert the conversion.
#define TIMEBASE_GPS_UNIX_OFFSET_S 315964800LL
#define TIMEBASE_GPS_LEAP_SECONDS  18LL

#ifdef __cplusplus
}
#endif

#endif
