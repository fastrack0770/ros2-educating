#pragma once

#include "wheeled_model_enhanced/action/reach_goal.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <atomic>
#include <mutex>
#include <memory>
#include <cmath>

class Degree;

class Radian
{
public:
    Radian(double value)
        : _value(value) {}

    Radian(Degree value);

    double value() const noexcept
    {
        return _value;
    }

private:
    double _value;
};

class Degree
{
public:
    Degree(double value)
        : _value(value) {}

    Degree(Radian value)
        : _value(value.value() * 180 / M_PI) {}

    double value() const noexcept { return _value; }

private:
    double _value;
};

inline Radian::Radian(Degree value) : _value(value.value() * M_PI / 180.0) {}

class Kilometer;

class Meter
{
public:
    Meter(double value)
        : _value(value) {}

    Meter(Kilometer value);

    double value() const noexcept { return _value; }

private:
    double _value;
};

class Kilometer
{
public:
    Kilometer(double value)
        : _value(value) {}

    Kilometer(Meter value)
        : _value(value.value() / 1000) {}

    double value() const noexcept { return _value; }

private:
    double _value;
};

inline Meter::Meter(Kilometer value) : _value(value.value() * 1000) {}

/**
 * Pos
 * Thread-safe GPS position holder
 */
class Pos
{
private:
    using ReachGoalAction = wheeled_model_enhanced::action::ReachGoal;

public:
    Pos() {}

    Pos(Degree latitude, Degree longitude, Meter altitude)
        : _latitude(latitude), _longitude(longitude), _altitude(altitude)
    {
    }

    Pos(std::shared_ptr<const ReachGoalAction::Goal> goal)
        : _latitude(goal->goal_lat), _longitude(goal->goal_long), _altitude(0.f)
    {
    }
    Radian latitude() const
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        return _latitude;
    }

    Radian longitude() const
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        return _longitude;
    }

    Meter altitude() const
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        return _altitude;
    }

    void setLatitude(Radian new_lat)
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        _latitude = new_lat;
    }

    void setLongitude(Radian new_long)
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        _longitude = new_long;
    }

    void setAltitude(Meter new_alt)
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        _altitude = new_alt;
    }

    Pos &operator=(const sensor_msgs::msg::NavSatFix &msg)
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        _latitude = msg.latitude;
        _longitude = msg.longitude;
        _altitude = msg.altitude;
        return *this;
    }

private:
    mutable std::mutex _m;

    Radian _latitude = 0.f;
    Radian _longitude = 0.f;
    Meter _altitude = 0.f;
};