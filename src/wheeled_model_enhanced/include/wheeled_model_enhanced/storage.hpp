#pragma once

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "types.hpp"
#include "utils.hpp"

#include <mutex>

/**
 * Storage
 * Keeps and calculates values related to robot movement
 */
class Storage
{
    struct PosExtended
    {
        Optional<Pos> gps_pos;
        Optional<Cartesian> topo_pos;
        Optional<Cartesian> related_pos; // the robot related pos is always zero
    };

  public:
    Optional<Pos> robot_gps_pos() const noexcept;
    Optional<Cartesian> robot_topo_pos() const noexcept;
    Optional<Cartesian> robot_related_pos() const noexcept;

    void set_robot_pos(const sensor_msgs::msg::NavSatFix &msg);

    Optional<Pos> waypoint_gps_pos() const noexcept;
    Optional<Cartesian> waypoint_topo_pos() const noexcept;
    Optional<Cartesian> waypoint_related_pos() const noexcept;

    void set_waypoint_pos(Pos);

    Meter distance_to_waypoint_gps() const noexcept;
    Meter distance_to_waypoint_related() const noexcept;

    void set_imu(const sensor_msgs::msg::Imu &imu);
    void set_odometry(const nav_msgs::msg::Odometry &new_data);

    Optional<double> angular_speed() const noexcept;
    Optional<bool> has_angular_speed() const noexcept;

    Optional<double> linear_speed() const noexcept;
    Optional<bool> has_linear_speed() const noexcept;

    /**
     * angular_acceleration
     * this value is used only for a reference due to an acceleration is not a constant
     */
    double angular_acceleration() const noexcept;
    Optional<double> linear_acceleration() const noexcept;

    Radian angle_to_waypoint() const noexcept;

    /**
     * robot_azimuth
     * Difference between the North and where the robot is facing
     */
    Optional<Radian> robot_azimuth() const noexcept;

    void set_robot_imu_twist(Radian twist);
    Radian robot_imu_twist() const noexcept;

    void set_robot_length(Meter length);
    Meter robot_length() const noexcept;

    /**
     * initialized
     * Return "true" when all data needed are provided
     */
    bool initialized() const noexcept;

  private:
    void calculate_waypoint_coords();
    void calculate_distances();
    void calculate_waypoint_angle();

  private:
    mutable std::recursive_mutex _m;

    Meter _distance_to_waypoint_gps = 0.f;
    Meter _distance_to_waypoint_related = 0.f;
    PosExtended _robot_pos;
    PosExtended _waypoint_pos;

    Optional<sensor_msgs::msg::Imu> _imu;
    Radian _angle_to_waypoint {0.f};

    Radian _robot_imu_twist {0};
    Meter _robot_length = 0;

    Optional<nav_msgs::msg::Odometry> _odometry;

    double _robot_angular_acceleration = 0.f;
    double _robot_linear_speed = 0.f;
};