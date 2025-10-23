#pragma once

#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "types.hpp"
#include "utils.hpp"

/**
 * Storage
 * Keeps values related to robot movement
 */
class Storage
{
    struct PosExtended
    {
        Pos gps_pos;
        Cartesian topo_pos;
        Cartesian related_pos; // robot related pos is always zero
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

        _robot_pos.topo_pos = utils::get_topo(_robot_pos.gps_pos, _robot_pos.gps_pos);

        _waypoint_pos.topo_pos = utils::get_topo(_waypoint_pos.gps_pos, _robot_pos.gps_pos);
        _waypoint_pos.related_pos = _waypoint_pos.topo_pos - _robot_pos.topo_pos;

        _distance_to_waypoint_gps = utils::distance(_robot_pos.gps_pos, _waypoint_pos.gps_pos);
        _distance_to_waypoint_related = utils::distance(_robot_pos.related_pos, _waypoint_pos.related_pos);
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

        _waypoint_pos.topo_pos = utils::get_topo(_waypoint_pos.gps_pos, _robot_pos.gps_pos);
        _waypoint_pos.related_pos = _waypoint_pos.topo_pos - _robot_pos.topo_pos;

        _distance_to_waypoint_gps = utils::distance(_robot_pos.gps_pos, _waypoint_pos.gps_pos);
        _distance_to_waypoint_related = utils::distance(_robot_pos.related_pos, _waypoint_pos.related_pos);
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
        // TODO refactor - imu_robot_twist must be defined only in one place
        _angle_to_waypoint =
            utils::get_angle_to_waypoint_signed(_robot_pos.related_pos, _waypoint_pos.related_pos, _imu, _robot_imu_twist);
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

  private:
    mutable std::recursive_mutex _m;

    Meter _distance_to_waypoint_gps = 0.f;
    Meter _distance_to_waypoint_related = 0.f;
    PosExtended _robot_pos;
    PosExtended _waypoint_pos;

    sensor_msgs::msg::Imu _imu;
    Radian _angle_to_waypoint = 0.f;

    // TODO make it as a parameter
    // Difference between the IMU sensor and the robot orientationss
    const Radian _robot_imu_twist = Degree(-90);
};