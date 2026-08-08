#ifndef GEO_H
#define GEO_H

// WGS84 geodetic <-> ECEF <-> ENU conversions.
//
// The fleet map needs one shared metric frame. Vehicles may report different
// GPS_GLOBAL_ORIGIN / HOME_POSITION values, so every vehicle origin is pushed
// through ECEF and back out into a single session ENU frame. Going through
// ECEF (rather than the flat-earth approximation used for the vehicle models)
// matters here: two origins a few km apart differ by tens of centimetres under
// the flat approximation, which is several map cells.

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GEO_WGS84_A  6378137.0                 // semi-major axis, metres
#define GEO_WGS84_F  (1.0 / 298.257223563)     // flattening
#define GEO_WGS84_E2 (GEO_WGS84_F * (2.0 - GEO_WGS84_F))

typedef struct {
    double lat_rad;
    double lon_rad;
    double alt_m;      // height above the ellipsoid reference used by the source
    double ecef[3];
    double sin_lat, cos_lat, sin_lon, cos_lon;
    bool   valid;
} geo_origin_t;

// Geodetic (degrees, metres) -> ECEF (metres).
void geo_lla_to_ecef(double lat_deg, double lon_deg, double alt_m, double ecef[3]);

// ECEF (metres) -> geodetic (degrees, metres). Bowring's method.
void geo_ecef_to_lla(const double ecef[3], double *lat_deg, double *lon_deg, double *alt_m);

// Establish a session origin. Safe to call repeatedly; last call wins.
void geo_origin_set(geo_origin_t *o, double lat_deg, double lon_deg, double alt_m);

// ECEF -> ENU (east, north, up) relative to the origin.
void geo_ecef_to_enu(const geo_origin_t *o, const double ecef[3], double enu[3]);

// ENU -> ECEF, the inverse of geo_ecef_to_enu.
void geo_enu_to_ecef(const geo_origin_t *o, const double enu[3], double ecef[3]);

// Geodetic -> ENU relative to the origin. This is the call sites' workhorse.
void geo_lla_to_enu(const geo_origin_t *o, double lat_deg, double lon_deg,
                    double alt_m, double enu[3]);

// ENU -> geodetic.
void geo_enu_to_lla(const geo_origin_t *o, const double enu[3],
                    double *lat_deg, double *lon_deg, double *alt_m);

// Great-circle-ish metric distance between two origins (metres, straight-line
// through ECEF). Used to decide whether two vehicles are plausibly co-located.
double geo_origin_separation_m(const geo_origin_t *a, const geo_origin_t *b);

// Fleet centroid of `count` origins, written into `out`. Averages in ECEF so
// the result is not skewed by the dateline or by high latitudes.
void geo_origin_centroid(const geo_origin_t *origins, int count, geo_origin_t *out);

#ifdef __cplusplus
}
#endif

#endif
