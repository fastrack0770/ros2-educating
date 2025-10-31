#pragma once

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "types.hpp"
#include "utils.hpp"

/**
 * Storage
 * Keeps and calculates values related to robot movement
 */
class Storage
{
    struct PosExtended
    {
        Pos gps_pos;
        Cartesian topo_pos;
        Cartesian related_pos; // the robot related pos is always zero
    };

  public:
    Pos robot_gps_pos() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        return _robot_pos.gps_pos;
    }

    Cartesian robot_topo_pos() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        return _robot_pos.topo_pos;
    }

    Cartesian robot_related_pos() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        return _robot_pos.related_pos;
    }

    void set_robot_pos(const sensor_msgs::msg::NavSatFix &msg)
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        _robot_pos.gps_pos = msg;

        // do related calculations
        _robot_pos.topo_pos = utils::get_topo(_robot_pos.gps_pos, _robot_pos.gps_pos);

        calculate_waypoint_coords();
        calculate_distances();
    }

    Pos waypoint_gps_pos() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        return _waypoint_pos.gps_pos;
    }

    Cartesian waypoint_topo_pos() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        return _waypoint_pos.topo_pos;
    }

    Cartesian waypoint_related_pos() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        return _waypoint_pos.related_pos;
    }

    void set_waypoint_pos(const sensor_msgs::msg::NavSatFix &msg)
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        _waypoint_pos.gps_pos = msg;

        // do related calculations
        calculate_waypoint_coords();
        calculate_distances();
    }

    Meter distance_to_waypoint_gps() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        return _distance_to_waypoint_gps;
    }

    void set_imu(const sensor_msgs::msg::Imu &imu)
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        _imu = imu;

        // do related calculations
        _angle_to_waypoint =
            utils::get_angle_to_waypoint_signed(_robot_pos.related_pos, _waypoint_pos.related_pos, robot_azimuth());
    }

    void set_odometry(const nav_msgs::msg::Odometry &new_data)
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        const auto angular_velocity_diff = new_data.twist.twist.angular.z - _odometry.twist.twist.angular.z;
        const auto passed_time =
            new_data.header.stamp.sec - _odometry.header.stamp.sec +
            static_cast<double>(new_data.header.stamp.nanosec - _odometry.header.stamp.nanosec) / 1'000'000'000;

        _odometry = new_data;

        // do related calculations
        _robot_angular_acceleration = angular_velocity_diff / passed_time;
    }

    double angular_speed() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        return _odometry.twist.twist.angular.z;
    }

    double linear_speed() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        return _odometry.twist.twist.linear.x;
    }

    /**
     * angular_acceleration
     * this value is used only for a reference due to an acceleration is not a constant
     */
    double angular_acceleration() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        return _robot_angular_acceleration;
    }

    double linear_acceleration() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        return _imu.linear_acceleration.x;
    }

    Radian angle_to_waypoint() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        return _angle_to_waypoint;
    }

    Meter distance_to_waypoint_related() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        return _distance_to_waypoint_related;
    }

    Radian robot_azimuth() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        return Radian(utils::get_euler_z_angle(_imu) + _robot_imu_twist.value());
    }

    void set_robot_imu_twist(Radian twist)
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        _robot_imu_twist = twist;

        // do related calculations
        _angle_to_waypoint =
            utils::get_angle_to_waypoint_signed(_robot_pos.related_pos, _waypoint_pos.related_pos, robot_azimuth());
    }

    void set_robot_length(Meter length)
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        _robot_length = length;
    }

  private:
    void calculate_waypoint_coords()
    {
        _waypoint_pos.topo_pos = utils::get_topo(_waypoint_pos.gps_pos, _robot_pos.gps_pos);
        _waypoint_pos.related_pos = _waypoint_pos.topo_pos - _robot_pos.topo_pos;
    }
    void calculate_distances()
    {
        _distance_to_waypoint_gps = utils::distance(_robot_pos.gps_pos, _waypoint_pos.gps_pos) - _robot_length;
        _distance_to_waypoint_related =
            utils::distance(_robot_pos.related_pos, _waypoint_pos.related_pos) - _robot_length;
    }

  private:
    mutable std::recursive_mutex _m;

    Meter _distance_to_waypoint_gps = 0.f;
    Meter _distance_to_waypoint_related = 0.f;
    PosExtended _robot_pos;
    PosExtended _waypoint_pos;

    sensor_msgs::msg::Imu _imu;
    Radian _angle_to_waypoint = 0.f;

    Radian _robot_imu_twist = Degree(90);
    Meter _robot_length = 1.5;

    nav_msgs::msg::Odometry _odometry;

    double _robot_angular_acceleration = 0.f;
};