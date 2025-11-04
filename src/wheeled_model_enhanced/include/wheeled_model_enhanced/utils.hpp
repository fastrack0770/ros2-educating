#pragma once

#include "types.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <cmath>

namespace utils
{

double to_deg(double value)
{
    return value / M_PI * 180;
}

double to_rad(double value)
{
    return value * M_PI / 180;
}

/**
 * abs
 * Get a module from a 3d vector. Result in meters
 */
double abs(Vector3D vec)
{
    return std::sqrt(std::pow(vec.x.value(), 2) + std::pow(vec.y.value(), 2) + std::pow(vec.z.value(), 2));
}

/**
 * make_vector
 * Make a vector from two cartesian coordinates
 */
Vector3D make_vector(const Cartesian &lhv, const Cartesian &rhv)
{
    return Vector3D(rhv.x - lhv.x, rhv.y - lhv.y, rhv.z - lhv.z);
}

/**
 * get_angle_between_vectors
 * Get angle between two 3D vectors
 */
double get_angle_between_vectors(const Vector3D &lhv, const Vector3D &rhv)
{
    return acos((lhv * rhv) / (abs(lhv) * abs(rhv)));
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
 * distance
 * Distance by Haversine formula
 * Source: https://www.movable-type.co.uk/scripts/latlong.html
 */
Meter distance(const Pos &lhv, const Pos &rhv)
{
    const double a = std::pow(std::sin(std::abs(lhv.latitude().value() - rhv.latitude().value()) / 2), 2) +
                     std::cos(lhv.latitude().value()) * std::cos(rhv.latitude().value()) *
                         std::pow(std::sin(std::abs(lhv.longitude().value() - rhv.longitude().value()) / 2), 2);
    const double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    const double earth_radius = earth_radius_at(lhv); // in meters
    const double d = earth_radius * c;

    return Meter(d);
}

/**
 * distance
 * Euclidean distance
 * Source: https://en.wikipedia.org/wiki/Euclidean_distance
 */
Meter distance(const Cartesian &lhv, const Cartesian &rhv)
{
    return Meter(
        sqrt(pow((lhv.x - rhv.x).value(), 2) + pow((lhv.y - rhv.y).value(), 2) + pow((lhv.z - rhv.z).value(), 2)));
}

/**
 * get_topo
 * ECEF to ENU transformation. Result is a topocentric coordinate related to the specified point of view.
 * Note that you must use only one point of view for every point you want to calculate, so all points
 * will be in the same cartesian coordinates system.
 */
Cartesian get_topo(const Pos &point, const Pos &point_of_view)
{
    // using wgs-84
    const double B = point.latitude().value();            // latitude, rad
    const double L = point.longitude().value();           // longitude, rad
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

        geocentric.x = Kilometer((N + H) * cos(B) * cos(L));      // North
        geocentric.y = Kilometer((N + H) * cos(B) * sin(L));      // Up
        geocentric.z = Kilometer((N + H - e_pow_2 * N) * sin(B)); // East
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

/**
 * get_euler_z_angle
 * Get angle, which to rotate to robot will face north
 * Resource: https://blog.endaq.com/quaternions-for-orientation
 */
double get_euler_z_angle(const sensor_msgs::msg::Imu &imu)
{
    const auto w = imu.orientation.w;
    const auto x = imu.orientation.x;
    const auto y = imu.orientation.y;
    const auto z = imu.orientation.z;
    return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

/**
 * get_angle_to_waypoint
 * Get angle, which to rotate to robot will face waypoint. The angle is positive wherever robot and waypoint are
 */
double get_angle_to_waypoint(const Cartesian &robot, const Cartesian &waypoint, const sensor_msgs::msg::Imu &imu)
{
    const auto wr_vec = make_vector(robot, waypoint); // Vector from robot to waypoint
    const auto rn_vec = make_vector(robot, Cartesian(robot.x + 100, robot.y, robot.z));

    const auto angle_to_north = get_euler_z_angle(imu);
    const auto rn_wr_angle = get_angle_between_vectors(rn_vec, wr_vec);

    return angle_to_north + rn_wr_angle;
}

Plane get_plane(const Point &point, const Vector3D &lhv, const Vector3D &rhv)
{
    const auto a = lhv.y * rhv.z - rhv.y * lhv.z;
    const auto b = lhv.x * rhv.z - rhv.x * lhv.z;
    const auto c = lhv.x * rhv.y - rhv.x * lhv.y;
    const auto d = (-1) * point.x * a + point.y * b - point.z * c;

    return {a, b, c, d};
}

Vector3D normalize(Vector3D vec)
{
    const auto m = abs(vec);
    return {vec.x / m, vec.y / m, vec.z / m};
}

/**
 * get_angle_between_vectors_signed
 * Get angle between two 3D vectors, positive when lhv to the left of rhv
 */
double get_angle_between_vectors_signed(const Vector3D &lhv, const Vector3D &rhv)
{
    return atan2((rhv ^ lhv) * Vector3D(0, 1, 0) /* the plane normal pointing up */, lhv * rhv);
}

/**
 * get_angle_to_waypoint_signed
 * Get angle, which to rotate to robot will face waypoint. The angle is positive when waypoint to the right of the robot
 */
Radian get_angle_to_waypoint_signed(const Cartesian &robot, const Cartesian &waypoint, const Radian &robot_azimuth)
{
    const auto wr_vec = make_vector(robot, waypoint); // Vector from robot to waypoint
    const auto rn_vec = make_vector(robot, Cartesian(robot.x + 100, robot.y, robot.z));

    auto rn_wr_angle = get_angle_between_vectors_signed(rn_vec, wr_vec);

    if (std::isnan(rn_wr_angle)) // angle between two vectors is 0
    {
        return robot_azimuth;
    }

    if (rn_wr_angle < 0)
    {
        rn_wr_angle += 2 * M_PI;
    }

    auto target_angle = Radian(robot_azimuth + rn_wr_angle).normalize();

    // finding the shortest way to turn
    if (target_angle < Degree(180))
    {
        target_angle += Radian(2 * M_PI);
    }
    if (target_angle > Degree(180))
    {
        target_angle -= Radian(2 * M_PI);
    }

    return target_angle;
}

/**
 * sign
 * Get a sign of a number.
 * Source: https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
 */
template <typename T> int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

namespace
{
using SpeedToSet = double;
using AccelerationDistance = double;
} // namespace
std::tuple<SpeedToSet, AccelerationDistance> get_speed(double max_speed, double acceleration, double desired_distance)
{
    double velocity_to_set = 0.f;
    double s_ac = pow(max_speed, 2) / (2 * acceleration); // distance, after which the velocity will become maximum
    if (desired_distance > 2 * s_ac)
    {
        velocity_to_set = max_speed * utils::sign(desired_distance);
    }
    else
    {
        s_ac = std::abs(desired_distance / 2);
        velocity_to_set = sqrt(2 * acceleration * s_ac) * utils::sign(desired_distance);
    }

    return {velocity_to_set, s_ac};
}

} // end of namespace utils