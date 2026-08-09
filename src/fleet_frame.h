#ifndef FLEET_FRAME_H
#define FLEET_FRAME_H

// One shared metric frame for the whole fleet.
//
// Vehicles may report different GPS_GLOBAL_ORIGIN / HOME_POSITION values, and
// some only ever report local NED against their own origin. Picking one session
// origin and pushing everything through ECEF is what makes the map merged
// rather than N maps drawn on top of each other.
//
// The session origin is frozen once the first ray goes in. Moving it afterwards
// would silently shift every cell already in the tree.

#include <stdbool.h>
#include <stdint.h>

#include "geo.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLEET_FRAME_MAX_VEHICLES 256

typedef enum {
    FLEET_ORIGIN_FIRST_SEEN = 0,   // first vehicle to report wins
    FLEET_ORIGIN_CENTROID,         // ECEF centroid of everything reported so far
    FLEET_ORIGIN_EXPLICIT,         // operator supplied it on the command line
} fleet_origin_policy_t;

typedef struct {
    geo_origin_t vehicle[FLEET_FRAME_MAX_VEHICLES];
    uint8_t      vehicle_source[FLEET_FRAME_MAX_VEHICLES];  // see FLEET_ORIGIN_SRC_*
    int          vehicle_count;

    geo_origin_t          session;
    fleet_origin_policy_t policy;
    bool                  frozen;      // no further origin changes accepted
    int                   origin_vehicle;  // which vehicle defined it (-1 = explicit/centroid)
} fleet_frame_t;

#define FLEET_ORIGIN_SRC_NONE       0
#define FLEET_ORIGIN_SRC_GPS_ORIGIN 1   // GPS_GLOBAL_ORIGIN
#define FLEET_ORIGIN_SRC_HOME       2   // HOME_POSITION
#define FLEET_ORIGIN_SRC_FIRST_FIX  3   // first global position seen

const char *fleet_origin_source_name(uint8_t src);

void fleet_frame_init(fleet_frame_t *ff, fleet_origin_policy_t policy);

// Set the session origin explicitly (command line). Freezes the policy.
void fleet_frame_set_explicit(fleet_frame_t *ff, double lat_deg, double lon_deg, double alt_m);

// Record a vehicle's own origin. Higher-quality sources overwrite lower ones.
// Returns true when the session origin moved as a result.
bool fleet_frame_note_origin(fleet_frame_t *ff, uint8_t vehicle_id, uint8_t src,
                             double lat_deg, double lon_deg, double alt_m);

// Stop accepting origin changes. Called before the first ray is inserted.
void fleet_frame_freeze(fleet_frame_t *ff);

bool fleet_frame_ready(const fleet_frame_t *ff);

// Global position -> session ENU. Independent of the vehicle's own origin.
bool fleet_frame_global_to_enu(const fleet_frame_t *ff, double lat_deg, double lon_deg,
                               double alt_m, double enu[3]);

// A vehicle's local NED (against *its* origin) -> session ENU. This is the call
// that makes two vehicles with different GPS_GLOBAL_ORIGIN land on one surface.
bool fleet_frame_local_ned_to_enu(const fleet_frame_t *ff, uint8_t vehicle_id,
                                  const double ned[3], double enu[3]);

// Session ENU -> Raylib world coordinates (X east, Y up, Z south).
static inline void fleet_enu_to_world(const double enu[3], float out[3]) {
    out[0] = (float)enu[0];
    out[1] = (float)enu[2];
    out[2] = (float)(-enu[1]);
}

static inline void fleet_world_to_enu(const float w[3], double out[3]) {
    out[0] = (double)w[0];
    out[1] = (double)(-w[2]);
    out[2] = (double)w[1];
}

#ifdef __cplusplus
}
#endif

#endif
