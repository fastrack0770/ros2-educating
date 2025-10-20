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

        _distance_to_waypoint = utils::distance_in_meters(_robot_pos.gps_pos, _waypoint_pos.gps_pos);
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
        
        _distance_to_waypoint = utils::distance_in_meters(_robot_pos.gps_pos, _waypoint_pos.gps_pos);
    }

    double distance_to_waypoint() const noexcept
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        return _distance_to_waypoint;
    }

  private:
    mutable std::mutex _m;

    double _distance_to_waypoint = 0.f;
    PosExtended _robot_pos;
    PosExtended _waypoint_pos;
};