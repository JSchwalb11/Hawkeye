#include "geo.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD_D (M_PI / 180.0)
#define RAD2DEG_D (180.0 / M_PI)

void geo_lla_to_ecef(double lat_deg, double lon_deg, double alt_m, double ecef[3]) {
    const double lat = lat_deg * DEG2RAD_D;
    const double lon = lon_deg * DEG2RAD_D;
    const double s = sin(lat);
    const double c = cos(lat);
    const double n = GEO_WGS84_A / sqrt(1.0 - GEO_WGS84_E2 * s * s);

    ecef[0] = (n + alt_m) * c * cos(lon);
    ecef[1] = (n + alt_m) * c * sin(lon);
    ecef[2] = (n * (1.0 - GEO_WGS84_E2) + alt_m) * s;
}

void geo_ecef_to_lla(const double ecef[3], double *lat_deg, double *lon_deg, double *alt_m) {
    const double x = ecef[0], y = ecef[1], z = ecef[2];
    const double b = GEO_WGS84_A * (1.0 - GEO_WGS84_F);
    const double ep2 = (GEO_WGS84_A * GEO_WGS84_A - b * b) / (b * b);
    const double p = sqrt(x * x + y * y);

    double lat, lon, alt;

    if (p < 1e-9) {
        // On the polar axis: Bowring's parametric latitude is undefined there.
        lon = 0.0;
        lat = (z >= 0.0) ? (M_PI * 0.5) : (-M_PI * 0.5);
        alt = fabs(z) - b;
    } else {
        const double th = atan2(GEO_WGS84_A * z, b * p);
        const double st = sin(th), ct = cos(th);
        lon = atan2(y, x);
        lat = atan2(z + ep2 * b * st * st * st,
                    p - GEO_WGS84_E2 * GEO_WGS84_A * ct * ct * ct);
        const double s = sin(lat);
        const double n = GEO_WGS84_A / sqrt(1.0 - GEO_WGS84_E2 * s * s);
        alt = p / cos(lat) - n;
    }

    if (lat_deg) *lat_deg = lat * RAD2DEG_D;
    if (lon_deg) *lon_deg = lon * RAD2DEG_D;
    if (alt_m)   *alt_m   = alt;
}

void geo_origin_set(geo_origin_t *o, double lat_deg, double lon_deg, double alt_m) {
    if (!o) return;
    memset(o, 0, sizeof(*o));
    o->lat_rad = lat_deg * DEG2RAD_D;
    o->lon_rad = lon_deg * DEG2RAD_D;
    o->alt_m   = alt_m;
    o->sin_lat = sin(o->lat_rad);
    o->cos_lat = cos(o->lat_rad);
    o->sin_lon = sin(o->lon_rad);
    o->cos_lon = cos(o->lon_rad);
    geo_lla_to_ecef(lat_deg, lon_deg, alt_m, o->ecef);
    o->valid = true;
}

void geo_ecef_to_enu(const geo_origin_t *o, const double ecef[3], double enu[3]) {
    if (!o || !o->valid) { enu[0] = enu[1] = enu[2] = 0.0; return; }

    const double dx = ecef[0] - o->ecef[0];
    const double dy = ecef[1] - o->ecef[1];
    const double dz = ecef[2] - o->ecef[2];

    enu[0] = -o->sin_lon * dx + o->cos_lon * dy;
    enu[1] = -o->sin_lat * o->cos_lon * dx - o->sin_lat * o->sin_lon * dy + o->cos_lat * dz;
    enu[2] =  o->cos_lat * o->cos_lon * dx + o->cos_lat * o->sin_lon * dy + o->sin_lat * dz;
}

void geo_enu_to_ecef(const geo_origin_t *o, const double enu[3], double ecef[3]) {
    if (!o || !o->valid) { ecef[0] = ecef[1] = ecef[2] = 0.0; return; }

    const double e = enu[0], n = enu[1], u = enu[2];

    ecef[0] = o->ecef[0] - o->sin_lon * e - o->sin_lat * o->cos_lon * n + o->cos_lat * o->cos_lon * u;
    ecef[1] = o->ecef[1] + o->cos_lon * e - o->sin_lat * o->sin_lon * n + o->cos_lat * o->sin_lon * u;
    ecef[2] = o->ecef[2] + o->cos_lat * n + o->sin_lat * u;
}

void geo_lla_to_enu(const geo_origin_t *o, double lat_deg, double lon_deg,
                    double alt_m, double enu[3]) {
    double ecef[3];
    geo_lla_to_ecef(lat_deg, lon_deg, alt_m, ecef);
    geo_ecef_to_enu(o, ecef, enu);
}

void geo_enu_to_lla(const geo_origin_t *o, const double enu[3],
                    double *lat_deg, double *lon_deg, double *alt_m) {
    double ecef[3];
    geo_enu_to_ecef(o, enu, ecef);
    geo_ecef_to_lla(ecef, lat_deg, lon_deg, alt_m);
}

double geo_origin_separation_m(const geo_origin_t *a, const geo_origin_t *b) {
    if (!a || !b || !a->valid || !b->valid) return 0.0;
    const double dx = a->ecef[0] - b->ecef[0];
    const double dy = a->ecef[1] - b->ecef[1];
    const double dz = a->ecef[2] - b->ecef[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

void geo_origin_centroid(const geo_origin_t *origins, int count, geo_origin_t *out) {
    if (!out) return;
    double sum[3] = {0.0, 0.0, 0.0};
    int n = 0;
    for (int i = 0; i < count; i++) {
        if (!origins[i].valid) continue;
        sum[0] += origins[i].ecef[0];
        sum[1] += origins[i].ecef[1];
        sum[2] += origins[i].ecef[2];
        n++;
    }
    if (n == 0) { memset(out, 0, sizeof(*out)); return; }

    const double mean[3] = { sum[0] / n, sum[1] / n, sum[2] / n };
    double lat, lon, alt;
    geo_ecef_to_lla(mean, &lat, &lon, &alt);
    geo_origin_set(out, lat, lon, alt);
}
