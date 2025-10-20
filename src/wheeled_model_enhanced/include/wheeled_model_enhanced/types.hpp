#pragma once

#include "wheeled_model_enhanced/action/reach_goal.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>

class Degree;

/**
 * Radian
 * To store value in radians
 */
class Radian
{
  public:
    Radian(double value) : _value(value)
    {
    }

    Radian(Degree value);

    double value() const noexcept
    {
        return _value;
    }

    bool operator==(const Radian &rhv) const noexcept
    {
        return _value == rhv._value;
    }

    bool operator!=(const Radian &rhv) const noexcept
    {
        return !(*this == rhv);
    }

    friend std::ostream &operator<<(std::ostream &os, const Radian &rhv);

  private:
    double _value;
};

inline std::ostream &operator<<(std::ostream &os, const Radian &rhv)
{
    return os << rhv._value;
}

/**
 * Degree
 * To store value in degrees
 */
class Degree
{
  public:
    Degree(double value) : _value(value)
    {
    }

    Degree(Radian value) : _value(value.value() * 180 / M_PI)
    {
    }

    double value() const noexcept
    {
        return _value;
    }

    bool operator==(const Degree &rhv) const noexcept
    {
        return _value == rhv._value;
    }

    bool operator!=(const Degree &rhv) const noexcept
    {
        return !(*this == rhv);
    }

  private:
    double _value;
};

inline Radian::Radian(Degree value) : _value(value.value() * M_PI / 180.0)
{
}

class Kilometer;

/**
 * Meter
 * To store value in meters
 */
class Meter
{
  public:
    Meter(double value) : _value(value)
    {
    }

    Meter(Kilometer value);

    double value() const noexcept
    {
        return _value;
    }

    friend Meter operator-(Meter lhv, const Meter &rhv)
    {
        return lhv._value - rhv._value;
    }

    friend Meter operator-(Meter lhv, double rhv)
    {
        return lhv._value - rhv;
    }

    friend Meter operator*(Meter lhv, double rhv)
    {
        return lhv._value * rhv;
    }

    Meter &operator-=(const Meter &rhv)
    {
        _value -= rhv._value;
        return *this;
    }

    friend Meter operator+(Meter lhv, const Meter &rhv)
    {
        return lhv._value + rhv._value;
    }

    friend Meter operator+(Meter lhv, double rhv)
    {
        return lhv._value + rhv;
    }

    Meter &operator+=(const Meter &rhv)
    {
        _value += rhv._value;
        return *this;
    }

    bool operator==(const Meter &rhv) const noexcept
    {
        return _value == rhv._value;
    }

    bool operator!=(const Meter &rhv) const noexcept
    {
        return !(*this == rhv);
    }

    friend double operator*(const double &lhv, const Meter &rhv);
    friend std::ostream &operator<<(std::ostream &os, const Meter &rhv);

  private:
    double _value;
};

inline double operator*(const double &lhv, const Meter &rhv)
{
    return lhv * rhv._value;
}

inline std::ostream &operator<<(std::ostream &os, const Meter &rhv)
{
    return os << rhv._value;
}

/**
 * Kilometer
 * To store value in km
 */
class Kilometer
{
  public:
    Kilometer(double value) : _value(value)
    {
    }

    Kilometer(Meter value) : _value(value.value() / 1000)
    {
    }

    double value() const noexcept
    {
        return _value;
    }

    bool operator==(const Kilometer &rhv) const noexcept
    {
        return _value == rhv._value;
    }

    bool operator!=(const Kilometer &rhv) const noexcept
    {
        return !(*this == rhv);
    }

  private:
    double _value;
};

inline Meter::Meter(Kilometer value) : _value(value.value() * 1000)
{
}

/**
 * Pos
 * GPS position holder
 */
class Pos
{
  private:
    using ReachGoalAction = wheeled_model_enhanced::action::ReachGoal;

  public:
    Pos()
    {
    }

    Pos(const Pos &copy)
    {
        _latitude = copy._latitude;
        _longitude = copy._longitude;
        _altitude = copy._altitude;
    }

    Pos(Radian latitude, Radian longitude, Meter altitude)
        : _latitude(latitude), _longitude(longitude), _altitude(altitude)
    {
    }

    Pos(std::shared_ptr<const ReachGoalAction::Goal> goal)
        : _latitude(Degree(goal->goal_lat)), _longitude(Degree(goal->goal_long)), _altitude(0.f)
    {
    }
    Radian latitude() const
    {
        return _latitude;
    }

    Radian longitude() const
    {
        return _longitude;
    }

    Meter altitude() const
    {
        return _altitude;
    }

    void setLatitude(Radian new_lat)
    {
        _latitude = new_lat;
    }

    void setLongitude(Radian new_long)
    {
        _longitude = new_long;
    }

    void setAltitude(Meter new_alt)
    {
        _altitude = new_alt;
    }

    Pos &operator=(const sensor_msgs::msg::NavSatFix &msg)
    {
        _latitude = msg.latitude * M_PI / 180;   // latitude in navsatfix in degrees
        _longitude = msg.longitude * M_PI / 180; // longitude in navsatfix in degrees
        _altitude = msg.altitude;
        return *this;
    }

    bool operator==(const Pos &rhv) const noexcept
    {
        return std::tie(_latitude, _longitude, _altitude) == std::tie(rhv._latitude, rhv._longitude, rhv._altitude);
    }

    bool operator!=(const Pos &rhv) const noexcept
    {
        return !(*this == rhv);
    }

    friend std::ostream &operator<<(std::ostream &os, const Pos &rhv);

  private:
    Radian _latitude = 0.f;
    Radian _longitude = 0.f;
    Meter _altitude = 0.f;
};

inline std::ostream &operator<<(std::ostream &os, const Pos &rhv)
{
    return os << "{ lat: " << rhv._latitude << " long: " << rhv._longitude << " alt: " << rhv._altitude << "}";
}

/**
 * Cartesian
 * To storage cartesian coordinates
 */
struct Cartesian
{
    Meter x = 0.f;
    Meter y = 0.f;
    Meter z = 0.f;

    Cartesian()
    {
    }
    Cartesian(Meter ix, Meter iy, Meter iz) : x(ix), y(iy), z(iz)
    {
    }

    Cartesian &operator-=(const Cartesian &rhv)
    {
        x -= rhv.x;
        y -= rhv.y;
        z -= rhv.z;

        return *this;
    }

    friend Cartesian operator-(Cartesian lhv, const Cartesian &rhv)
    {
        lhv -= rhv;

        return lhv;
    }

    Cartesian &operator+=(const Cartesian &rhv)
    {
        x += rhv.x;
        y += rhv.y;
        z += rhv.z;

        return *this;
    }

    friend Cartesian operator+(Cartesian lhv, const Cartesian &rhv)
    {
        lhv += rhv;

        return lhv;
    }

    bool operator==(const Cartesian &rhv) const noexcept
    {
        return std::tie(x, y, z) == std::tie(rhv.x, rhv.y, rhv.z);
    }

    bool operator!=(const Cartesian &rhv) const noexcept
    {
        return !(*this == rhv);
    }
};

inline std::ostream &operator<<(std::ostream &os, const Cartesian &rhv)
{
    return os << "{ x: " << rhv.x << " y: " << rhv.y << " z: " << rhv.z << "}";
}