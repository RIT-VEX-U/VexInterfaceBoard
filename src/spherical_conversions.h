#ifndef SPHERICAL_CONVERSIONS_H_
#define SPHERICAL_CONVERSIONS_H_

#define EIGEN_NO_DEBUG

struct c_point {
    double x;
    double y;
    double z;
};

struct g_point {
    double lat;
    double lon;
    double height;
};

double nmod(double x, double y);
double to_radians(double degrees);
double to_degrees(double radians);
double to_geodetic_inclination_degrees(double degrees);
double to_geodetic_inclination_radians(double radians);
double to_spherical_inclination_degrees(double degrees);
double to_spherical_inclination_radians(double radians);
double to_geodetic_azimuth_degrees(double degrees);
double to_geodetic_azimuth_radians(double radians);
double to_spherical_azimuth_degrees(double degrees);
double to_spherical_azimuth_radians(double radians);

c_point geodetic_to_ecef(g_point pose);
c_point geodetic_to_enu(g_point ref_geo, g_point geo_point);

c_point ecef_to_enu(c_point ref_ecef, c_point ecef_point);
g_point ecef_to_geodetic(c_point ecef);

c_point enu_to_ecef(c_point ref_ecef, c_point enu_point);
g_point enu_to_geodetic(c_point ref_ecef, c_point enu_point);
g_point enu_to_geodetic(g_point ref_geo, g_point geo_point);


#endif // SPHERICAL_CONVERSIONS_H_
