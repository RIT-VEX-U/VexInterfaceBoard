#include <cmath>
#include "spherical_conversions.h"

double nmod(double x, double y) {
    return fmod(fmod(x, y) + y, y);
}

double to_radians(double degrees) {
    return degrees * (M_PI / 180);
}

double to_degrees(double radians) {
    return radians * (180.0 / M_PI);
}

double to_geodetic_inclination_degrees(double degrees) {
    return -1 * (degrees - 90);
}

double to_geodetic_inclination_radians(double radians) {
    return -1 * (radians - (M_PI / 2));
}

double to_spherical_inclination_degrees(double degrees) {
    return (degrees * -1) + 90;
}

double to_spherical_inclination_radians(double radians) {
    return (radians * -1) + (M_PI / 2);
}

double to_geodetic_azimuth_degrees(double degrees) {
    return degrees > 180 ? degrees - 360 : degrees;
}

double to_geodetic_azimuth_radians(double radians) {
    return radians > M_PI ? radians - 2 * M_PI : radians;
}

double to_spherical_azimuth_degrees(double degrees) {
    return nmod((degrees + 360), 360);
}

double to_spherical_azimuth_radians(double radians) {
    return nmod((radians + 2 * M_PI), (2 * M_PI));
}

c_point geodetic_to_ecef(g_point pose) {
    c_point out;

    double lat = to_spherical_inclination_radians(to_radians(pose.lat));
    double lon = to_spherical_azimuth_radians(to_radians(pose.lon));
    double height = pose.height + 6371000;

    double sinlat = sin(lat);
    double sinlon = sin(lon);
    double coslat = cos(lat);
    double coslon = cos(lon);

    out.x = height * sinlat * coslon;
    out.y = height * sinlat * sinlon;
    out.z = height * coslat;

    return out;
}

c_point geodetic_to_enu(g_point ref_geo, g_point geo_point) {
    c_point ref_ecef = geodetic_to_ecef(ref_geo);
    c_point ecef_point = geodetic_to_ecef(geo_point);
    return ecef_to_enu(ref_ecef, ecef_point);
}

c_point ecef_to_enu(c_point ref_ecef, c_point ecef_point) {
    c_point enu;
    g_point ref_geo = ecef_to_geodetic(ref_ecef);


    double lat = to_spherical_inclination_radians(to_radians(ref_geo.lat));
    double lon = to_spherical_azimuth_radians(to_radians(ref_geo.lon));

    double sinlat{sin(lat)};
    double coslat{cos(lat)};
    double sinlon{sin(lon)};
    double coslon{cos(lon)};

    // Relative ECEF coordinates
    double dx = ecef_point.x - ref_ecef.x;
    double dy = ecef_point.y - ref_ecef.y;
    double dz = ecef_point.z - ref_ecef.z;

    enu.x = -sinlon * dx + coslon * dy;
    enu.y = -sinlat * coslon * dx + (-sinlat * sinlon * dy) + (coslat * dz);
    enu.z = coslat * coslon * dx + coslat * sinlon * dy + sinlat * dz;

    return enu;
}

g_point ecef_to_geodetic(c_point ecef) {
    g_point out;
    double r = sqrt(ecef.x * ecef.x + ecef.y * ecef.y + ecef.z * ecef.z);
    double theta = copysign(1.0, ecef.z) * acos(ecef.z / r);
    double phi = copysign(1.0, ecef.y) * acos(ecef.x / sqrt(ecef.x * ecef.x + ecef.y * ecef.y));
    out.height = r - 6371000;
    out.lat = to_geodetic_inclination_degrees(to_degrees(theta));
    out.lon = to_geodetic_azimuth_degrees(to_degrees(phi));

    return out;
}

c_point enu_to_ecef(c_point ref_ecef, c_point enu_point) {
    c_point ecef;
    g_point ref_geo = ecef_to_geodetic(ref_ecef);
    double lat = to_spherical_inclination_radians(to_radians(ref_geo.lat));
    double lon = to_spherical_azimuth_radians(to_radians(ref_geo.lon));

    double sinlat{sin(lat)};
    double coslat{cos(lat)};
    double sinlon{sin(lon)};
    double coslon{cos(lon)};

    ecef.x = (-sinlon * enu_point.x - sinlat * coslon * enu_point.y + coslat * coslon * enu_point.z) + ref_ecef.x;
    ecef.y = (coslon * enu_point.x - sinlat * sinlon * enu_point.y + coslat * sinlon * enu_point.z) + ref_ecef.y;
    ecef.z = (coslat * enu_point.y + sinlat * enu_point.z) + ref_ecef.z;

    return ecef;
}

g_point enu_to_geodetic(c_point ref_ecef, c_point enu_point) {
    c_point ecef = enu_to_ecef(ref_ecef, enu_point);
    return ecef_to_geodetic(ecef);
}

g_point enu_to_geodetic(g_point ref_geo, g_point geo_point) {
    c_point ref_ecef = geodetic_to_ecef(ref_geo);
    c_point enu_point = geodetic_to_ecef(geo_point);
    c_point ecef_point = enu_to_ecef(ref_ecef, enu_point);
    return ecef_to_geodetic(ecef_point);
}
