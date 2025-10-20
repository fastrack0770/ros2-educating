#pragma once

#include "types.hpp"

#include <cmath>

namespace utils
{
/**
 * get_vector_3d_abs
 * Get a module from a 3d vector
 */
double get_vector_3d_abs(double x, double y, double z)
{
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
}

/**
 * earth_radius_at
 * Earth radius at the specified latitude
 * Realization got from: https://gis.stackexchange.com/questions/242188/calculating-the-earth-radius-at-latitude
 */
double earth_radius_at(const Pos &pos)
{
    constexpr double WGS_ELLIPSOID_EQ = 6378137.0;    // equatorial, in meters
    constexpr double WGS_ELLIPSOID_POL = 6356752.314; // polar, in meters

    const auto f1 = std::pow(std::pow(WGS_ELLIPSOID_EQ, 2) * std::cos(pos.latitude().value()), 2);
    const auto f2 = std::pow(std::pow(WGS_ELLIPSOID_POL, 2) * std::sin(pos.latitude().value()), 2);
    const auto f3 = std::pow(WGS_ELLIPSOID_EQ * std::cos(pos.latitude().value()), 2);
    const auto f4 = std::pow(WGS_ELLIPSOID_POL * std::sin(pos.latitude().value()), 2);

    const auto radius = std::sqrt((f1 + f2) / (f3 + f4));

    return radius;
}

/**
 * earth_radius_at_2
 * Earth radius at the specified latitude, an alternative method
 * Realization got from: https://gscommunitycodes.usf.edu/geoscicommunitycodes/public/geophysics/Gravity/earth_shape.php
 */
double earth_radius_at_2(const Pos &pos)
{
    constexpr double f = 1 / 298.257223563;
    constexpr double WGS_ELLIPSOID_EQ = 6378137.0; // equatorial, in meters

    const auto radius = WGS_ELLIPSOID_EQ * (1 - f * std::pow(std::sin(pos.latitude().value()), 2));

    return radius;
}

/**
 * distance_in_meters
 * Distance by Haversine formula
 * Source: https://www.movable-type.co.uk/scripts/latlong.html
 */
double distance_in_meters(const Pos &lhv, const Pos &rhv)
{
    const double a = std::pow(std::sin(std::abs(lhv.latitude().value() - rhv.latitude().value()) / 2), 2) +
                     std::cos(lhv.latitude().value()) * std::cos(rhv.latitude().value()) *
                         std::pow(std::sin(std::abs(lhv.longitude().value() - rhv.longitude().value()) / 2), 2);
    const double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    const double earth_radius = earth_radius_at(lhv); // in meters
    const double d = earth_radius * c;

    return d;
}

/**
 * get_topo
 * Get topocentric coordinates related to the specified point of view
 */
Cartesian get_topo(const Pos &point, const Pos &point_of_view)
{
    // using wgs-84
    const double B = point.latitude().value();        // latitude, rad
    const double L = point.longitude().value();      g // longitude, rad
    const double H = Kilometer(point.altitude()).value(); // altitude, km

    const double B_view = point_of_view.latitude().value();  // latitude, rad
    const double L_view = point_of_view.longitude().value(); // longitude, rad

    // geodesic to geocentric
    Cartesian geocentric;
    {
        const double f = 1 / 298.257234;
        const double a = 6378.137; // equatorial earth radius in km
        const double e_pow_2 = sqrt(f * (2 - f));
        const double N = a / sqrt(1 - e_pow_2 * pow(sin(B), 2));

        geocentric.x = Kilometer((N + H) * cos(B) * cos(L));
        geocentric.y = Kilometer((N + H) * cos(B) * sin(L));
        geocentric.z = Kilometer((N + H - e_pow_2 * N) * sin(B));
    }

    // geocentric to topocentric
    Cartesian topocentric;
    {
        /**    (-sinB * cosL   -sinB * sinL   cosB)
         * M = ( cosB * cosL    cosB * sinL   sinB)
         *     (-sinL           cosL          0)
         */
        topocentric.x = -1 * sin(B_view) * cos(L_view) * geocentric.x - sin(B_view) * sin(L_view) * geocentric.y +
                        cos(B_view) * geocentric.z;
        topocentric.y = cos(B_view) * cos(L_view) * geocentric.x + cos(B_view) * sin(L_view) * geocentric.y +
                        sin(B_view) * geocentric.z;
        topocentric.z = -1 * sin(L_view) * geocentric.x + cos(L_view) * geocentric.y;
    }

    return topocentric;
}
} // end of namespace utils