#include "wheeled_model_enhanced/storage.hpp"

Optional<Pos> Storage::robot_gps_pos() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    return _robot_pos.gps_pos;
}

Optional<Cartesian> Storage::robot_topo_pos() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    return _robot_pos.topo_pos;
}

Optional<Cartesian> Storage::robot_related_pos() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    return _robot_pos.related_pos;
}

void Storage::set_robot_pos(const sensor_msgs::msg::NavSatFix &msg)
{
    const std::lock_guard<decltype(_m)> lock(_m);

    _robot_pos.gps_pos = msg;

    // do related calculations
    _robot_pos.topo_pos = utils::get_topo(_robot_pos.gps_pos.value(), _robot_pos.gps_pos.value());
    _robot_pos.related_pos = Cartesian(0, 0, 0);

    calculate_waypoint_coords();
    calculate_distances();
}

Optional<Pos> Storage::waypoint_gps_pos() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    return _waypoint_pos.gps_pos;
}

Optional<Cartesian> Storage::waypoint_topo_pos() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    return _waypoint_pos.topo_pos;
}

Optional<Cartesian> Storage::waypoint_related_pos() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    return _waypoint_pos.related_pos;
}

void Storage::set_waypoint_pos(Pos msg)
{
    const std::lock_guard<decltype(_m)> lock(_m);

    _waypoint_pos.gps_pos = msg;

    calculate_waypoint_coords();
    calculate_distances();
    calculate_waypoint_angle();
}

Meter Storage::distance_to_waypoint_gps() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    return _distance_to_waypoint_gps;
}

void Storage::set_imu(const sensor_msgs::msg::Imu &imu)
{
    const std::lock_guard<decltype(_m)> lock(_m);

    _imu = imu;

    calculate_waypoint_angle();
}

void Storage::set_odometry(const nav_msgs::msg::Odometry &new_data)
{
    const std::lock_guard<decltype(_m)> lock(_m);

    if (not _odometry.has_value())
    {
        _odometry = new_data;
        return;
    }

    const auto angular_velocity_diff = new_data.twist.twist.angular.z - _odometry->twist.twist.angular.z;
    const auto passed_time = new_data.header.stamp - _odometry->header.stamp;

    _odometry = new_data;

    // do related calculations
    _robot_angular_acceleration = angular_velocity_diff / passed_time;
}

Optional<double> Storage::angular_speed() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    if (not _odometry.has_value())
    {
        return Nullopt;
    }

    return _odometry->twist.twist.angular.z;
}

Optional<bool> Storage::has_angular_speed() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    const auto ang_speed = angular_speed();
    if (not ang_speed.has_value())
    {
        return Nullopt;
    }

    return abs(ang_speed.value()) > 0;
}

Optional<double> Storage::linear_speed() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    if (not _odometry.has_value())
    {
        return Nullopt;
    }

    return _odometry->twist.twist.linear.x;
}

Optional<bool> Storage::has_linear_speed() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    const auto lin_speed = linear_speed();
    if (not lin_speed.has_value())
    {
        return Nullopt;
    }

    return abs(lin_speed.value()) > 0;
}

/**
 * angular_acceleration
 * this value is used only for a reference due to an acceleration is not a constant
 */
double Storage::angular_acceleration() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    return _robot_angular_acceleration;
}

Optional<double> Storage::linear_acceleration() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    if (not _imu.has_value())
    {
        return Nullopt;
    }

    return _imu->linear_acceleration.x;
}

Radian Storage::angle_to_waypoint() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    return _angle_to_waypoint;
}

Meter Storage::distance_to_waypoint_related() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    return _distance_to_waypoint_related;
}

/**
 * robot_azimuth
 * Difference between the North and where the robot is facing
 */
Optional<Radian> Storage::robot_azimuth() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    if (not _imu.has_value())
    {
        return Nullopt;
    }

    return Radian(utils::get_euler_z_angle(_imu.value()) + _robot_imu_twist.to_double());
}

void Storage::set_robot_imu_twist(Radian twist)
{
    const std::lock_guard<decltype(_m)> lock(_m);

    _robot_imu_twist = twist;

    calculate_waypoint_angle();
}

Radian Storage::robot_imu_twist() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    return _robot_imu_twist;
}

void Storage::set_robot_length(Meter length)
{
    const std::lock_guard<decltype(_m)> lock(_m);

    _robot_length = length;
}

Meter Storage::robot_length() const noexcept
{
    const std::lock_guard<decltype(_m)> lock(_m);

    return _robot_length;
}

void Storage::calculate_waypoint_coords()
{
    if (_waypoint_pos.gps_pos.has_value() and _robot_pos.gps_pos.has_value())
    {
        _waypoint_pos.topo_pos = utils::get_topo(_waypoint_pos.gps_pos.value(), _robot_pos.gps_pos.value());
    }

    if (_waypoint_pos.topo_pos.has_value() and _robot_pos.topo_pos.has_value())
    {
        _waypoint_pos.related_pos = _waypoint_pos.topo_pos.value() - _robot_pos.topo_pos.value();
    }
}
void Storage::calculate_distances()
{
    if (_waypoint_pos.gps_pos.has_value() and _robot_pos.gps_pos.has_value())
    {
        _distance_to_waypoint_gps =
            utils::distance(_robot_pos.gps_pos.value(), _waypoint_pos.gps_pos.value()) - _robot_length;
    }

    if (_robot_pos.related_pos.has_value() and _waypoint_pos.related_pos.has_value())
    {
        _distance_to_waypoint_related =
            utils::distance(_robot_pos.related_pos.value(), _waypoint_pos.related_pos.value()) - _robot_length;
    }
}

void Storage::calculate_waypoint_angle()
{
    const auto robot_az = robot_azimuth();
    if (_waypoint_pos.related_pos.has_value() and _robot_pos.related_pos.has_value() and robot_az.has_value())
    {
        _angle_to_waypoint = utils::get_angle_to_waypoint_signed(_robot_pos.related_pos.value(),
                                                                 _waypoint_pos.related_pos.value(), robot_az.value());
    }
}

bool Storage::initialized() const noexcept
{
    return _robot_pos.gps_pos.has_value() and _robot_pos.related_pos.has_value() and _robot_pos.topo_pos.has_value() and
           _imu.has_value() and _odometry.has_value();
}