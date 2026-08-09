#include "timebase.h"

#include <string.h>

static const char *const k_prov_name[TIME_PROV_COUNT] = {
    "none",
    "boot?",
    "boot/manual",
    "arrival",
    "TIMESYNC",
    "GPS raw",
    "GPS UTC",
};

static const char *const k_prov_detail[TIME_PROV_COUNT] = {
    "no time reference seen; timestamps are raw source units",
    "boot-relative stamps, offset assumed zero (vehicles may be seconds apart)",
    "boot-relative stamps, offset set by the operator",
    "aligned on wall-clock arrival (tlog writer, or the live receive path)",
    "aligned on TIMESYNC round-trip against the viewer clock",
    "aligned on GPS_RAW_INT.time_usec (GPS epoch, leap-second corrected)",
    "aligned on SYSTEM_TIME.time_unix_usec (UTC from the vehicle)",
};

const char *timebase_provenance_name(time_provenance_t p) {
    if (p < 0 || p >= TIME_PROV_COUNT) return "?";
    return k_prov_name[p];
}

const char *timebase_provenance_detail(time_provenance_t p) {
    if (p < 0 || p >= TIME_PROV_COUNT) return "unknown time provenance";
    return k_prov_detail[p];
}

void timebase_session_init(timebase_session_t *s) {
    if (s) memset(s, 0, sizeof(*s));
}

void timebase_init(timebase_t *tb) {
    if (!tb) return;
    memset(tb, 0, sizeof(*tb));
    tb->provenance = TIME_PROV_NONE;
    tb->rtt_ns_min = INT64_MAX;
}

void timebase_session_note(timebase_session_t *s, int64_t session_ns) {
    if (!s) return;
    if (!s->have_horizon || session_ns > s->horizon_ns) {
        s->horizon_ns = session_ns;
        s->have_horizon = true;
    }
}

// Adopt `unix_ns` as the session epoch the first time anything absolute shows
// up. Every later source aligns against that same epoch, which is what makes
// two vehicles land on one timeline.
static void session_adopt_epoch(timebase_session_t *s, int64_t unix_ns) {
    if (!s || s->epoch_known) return;
    s->epoch_unix_ns = unix_ns;
    s->epoch_known = true;
}

static bool apply_offset(timebase_t *tb, int64_t offset_ns, time_provenance_t prov) {
    if (tb->locked) return false;
    // Only ever upgrade. A late boot-relative sample must not clobber a GPS lock.
    if (prov < tb->provenance) return false;
    const bool changed = (tb->offset_ns != offset_ns) || (tb->provenance != prov);
    tb->offset_ns  = offset_ns;
    tb->provenance = prov;
    return changed;
}

static void note_source(timebase_t *tb, int64_t source_ns) {
    if (!tb->have_first) {
        tb->first_source_ns = source_ns;
        tb->have_first = true;
    }
    tb->last_source_ns = source_ns;
}

bool timebase_observe_system_time(timebase_session_t *s, timebase_t *tb,
                                  uint64_t time_boot_ms, uint64_t time_unix_usec) {
    if (!s || !tb) return false;
    if (time_unix_usec == 0) return false;   // PX4 sends 0 until it has UTC

    const int64_t source_ns = (int64_t)time_boot_ms * 1000000LL;
    const int64_t unix_ns   = (int64_t)time_unix_usec * 1000LL;
    note_source(tb, source_ns);
    session_adopt_epoch(s, unix_ns);

    // session = source + offset, and session = unix - epoch.
    const int64_t offset = (unix_ns - s->epoch_unix_ns) - source_ns;
    return apply_offset(tb, offset, TIME_PROV_GPS_UNIX);
}

bool timebase_observe_gps_raw(timebase_session_t *s, timebase_t *tb,
                              uint64_t source_time_usec, uint64_t gps_time_usec) {
    if (!s || !tb) return false;
    if (gps_time_usec == 0) return false;

    const int64_t source_ns = (int64_t)source_time_usec * 1000LL;
    const int64_t unix_ns =
        (int64_t)gps_time_usec * 1000LL
        + (TIMEBASE_GPS_UNIX_OFFSET_S - TIMEBASE_GPS_LEAP_SECONDS) * 1000000000LL;

    note_source(tb, source_ns);
    session_adopt_epoch(s, unix_ns);

    const int64_t offset = (unix_ns - s->epoch_unix_ns) - source_ns;
    return apply_offset(tb, offset, TIME_PROV_GPS_RAW);
}

bool timebase_observe_timesync(timebase_session_t *s, timebase_t *tb,
                               int64_t ts1_ns, int64_t tc1_ns, int64_t local_ns) {
    if (!s || !tb) return false;
    // In a reply, ts1 is the echo of *our* send stamp and tc1 is the vehicle's
    // clock. Mixing them up subtracts a boot-relative clock from a wall clock
    // and yields an offset decades wide -- which then outranks and destroys a
    // perfectly good arrival alignment.
    if (ts1_ns == 0 || tc1_ns == 0) return false;

    const int64_t rtt = local_ns - ts1_ns;
    if (rtt < 0) return false;
    // A round trip longer than a minute is not a measurement, it is a clock
    // that does not belong to us.
    if (rtt > 60000000000LL) return false;

    tb->rtt_ns = rtt;
    if (rtt < tb->rtt_ns_min) tb->rtt_ns_min = rtt;
    tb->timesync_count++;

    // The vehicle stamped tc1 at roughly the midpoint of the round trip.
    const int64_t local_at_remote = ts1_ns + rtt / 2;
    note_source(tb, tc1_ns);
    session_adopt_epoch(s, local_at_remote);

    const int64_t offset = (local_at_remote - s->epoch_unix_ns) - tc1_ns;
    return apply_offset(tb, offset, TIME_PROV_TIMESYNC);
}

bool timebase_observe_arrival(timebase_session_t *s, timebase_t *tb,
                              int64_t source_ns, int64_t arrival_unix_ns) {
    if (!s || !tb) return false;

    note_source(tb, source_ns);
    session_adopt_epoch(s, arrival_unix_ns);

    const int64_t offset = (arrival_unix_ns - s->epoch_unix_ns) - source_ns;
    return apply_offset(tb, offset, TIME_PROV_TLOG_ARRIVAL);
}

void timebase_set_manual(timebase_t *tb, int64_t offset_ns) {
    if (!tb) return;
    tb->offset_ns  = offset_ns;
    tb->provenance = TIME_PROV_BOOT_MANUAL;
    tb->locked     = true;
}

void timebase_unlock(timebase_t *tb) {
    if (tb) tb->locked = false;
}

int64_t timebase_to_session(const timebase_t *tb, int64_t source_ns) {
    if (!tb) return source_ns;
    return source_ns + tb->offset_ns;
}

int64_t timebase_from_session(const timebase_t *tb, int64_t session_ns) {
    if (!tb) return session_ns;
    return session_ns - tb->offset_ns;
}

time_provenance_t timebase_fleet_worst(const timebase_t *const *tbs, int count) {
    time_provenance_t worst = TIME_PROV_GPS_UNIX;
    bool any = false;
    for (int i = 0; i < count; i++) {
        if (!tbs[i]) continue;
        any = true;
        if (tbs[i]->provenance < worst) worst = tbs[i]->provenance;
    }
    return any ? worst : TIME_PROV_NONE;
}

int64_t timebase_fleet_spread_ns(const timebase_t *const *tbs, int count) {
    int64_t lo = INT64_MAX, hi = INT64_MIN;
    for (int i = 0; i < count; i++) {
        if (!tbs[i] || tbs[i]->provenance == TIME_PROV_NONE) continue;
        if (tbs[i]->offset_ns < lo) lo = tbs[i]->offset_ns;
        if (tbs[i]->offset_ns > hi) hi = tbs[i]->offset_ns;
    }
    if (lo == INT64_MAX) return 0;
    return hi - lo;
}
