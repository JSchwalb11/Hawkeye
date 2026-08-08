#include "fleet_frame.h"

#include <string.h>

static const char *const k_src_name[] = {
    "none", "GPS_GLOBAL_ORIGIN", "HOME_POSITION", "first fix",
};

const char *fleet_origin_source_name(uint8_t src) {
    if (src >= (uint8_t)(sizeof(k_src_name) / sizeof(k_src_name[0]))) return "?";
    return k_src_name[src];
}

void fleet_frame_init(fleet_frame_t *ff, fleet_origin_policy_t policy) {
    if (!ff) return;
    memset(ff, 0, sizeof(*ff));
    ff->policy = policy;
    ff->origin_vehicle = -1;
}

void fleet_frame_set_explicit(fleet_frame_t *ff, double lat_deg, double lon_deg, double alt_m) {
    if (!ff) return;
    geo_origin_set(&ff->session, lat_deg, lon_deg, alt_m);
    ff->policy = FLEET_ORIGIN_EXPLICIT;
    ff->origin_vehicle = -1;
}

static void recompute_centroid(fleet_frame_t *ff) {
    geo_origin_t centroid;
    geo_origin_centroid(ff->vehicle, FLEET_FRAME_MAX_VEHICLES, &centroid);
    if (centroid.valid) ff->session = centroid;
}

bool fleet_frame_note_origin(fleet_frame_t *ff, uint8_t vehicle_id, uint8_t src,
                             double lat_deg, double lon_deg, double alt_m) {
    if (!ff) return false;
    if (src == FLEET_ORIGIN_SRC_NONE) return false;

    // Ignore the null island: a vehicle without a fix reports 0/0 and adopting
    // it would put the session origin off the coast of Ghana.
    if (lat_deg == 0.0 && lon_deg == 0.0) return false;

    geo_origin_t *slot = &ff->vehicle[vehicle_id];
    if (slot->valid && ff->vehicle_source[vehicle_id] > src) return false;

    if (!slot->valid) ff->vehicle_count++;
    geo_origin_set(slot, lat_deg, lon_deg, alt_m);
    ff->vehicle_source[vehicle_id] = src;

    if (ff->frozen || ff->policy == FLEET_ORIGIN_EXPLICIT) return false;

    const double before[3] = { ff->session.ecef[0], ff->session.ecef[1], ff->session.ecef[2] };
    const bool had = ff->session.valid;

    if (ff->policy == FLEET_ORIGIN_CENTROID) {
        recompute_centroid(ff);
        ff->origin_vehicle = -1;
    } else if (!had) {
        ff->session = *slot;
        ff->origin_vehicle = (int)vehicle_id;
    }

    if (!had) return ff->session.valid;
    return ff->session.ecef[0] != before[0]
        || ff->session.ecef[1] != before[1]
        || ff->session.ecef[2] != before[2];
}

void fleet_frame_freeze(fleet_frame_t *ff) {
    if (ff) ff->frozen = true;
}

bool fleet_frame_ready(const fleet_frame_t *ff) {
    return ff && ff->session.valid;
}

bool fleet_frame_global_to_enu(const fleet_frame_t *ff, double lat_deg, double lon_deg,
                               double alt_m, double enu[3]) {
    if (!ff || !ff->session.valid) return false;
    geo_lla_to_enu(&ff->session, lat_deg, lon_deg, alt_m, enu);
    return true;
}

bool fleet_frame_local_ned_to_enu(const fleet_frame_t *ff, uint8_t vehicle_id,
                                  const double ned[3], double enu[3]) {
    if (!ff || !ff->session.valid) return false;
    const geo_origin_t *vo = &ff->vehicle[vehicle_id];
    if (!vo->valid) return false;

    // The vehicle's NED offset is expressed in *its* local tangent plane, so it
    // goes out to ECEF through that plane and back into the session plane.
    const double local_enu[3] = { ned[1], ned[0], -ned[2] };
    double ecef[3];
    geo_enu_to_ecef(vo, local_enu, ecef);
    geo_ecef_to_enu(&ff->session, ecef, enu);
    return true;
}
