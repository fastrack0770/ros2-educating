#pragma once

#include "wheeled_model_enhanced/action/reach_goal.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>

class Degree;

template <typename T> class Optional;

/**
 * Radian
 * To store value in radians
 */
class Radian
{
  public:
    constexpr Radian(double value) : _value(value)
    {
    }

    constexpr Radian(Degree value);

    constexpr double value() const noexcept
    {
        return _value;
    }

    constexpr friend Radian operator+(Radian lhv, const Radian &rhv)
    {
        return lhv._value + rhv._value;
    }

    constexpr Radian &operator+=(const Radian &rhv)
    {
        _value += rhv._value;
        return *this;
    }

    constexpr friend Radian operator-(Radian lhv, const Radian &rhv)
    {
        return lhv._value - rhv._value;
    }

    constexpr Radian &operator-=(const Radian &rhv)
    {
        _value -= rhv._value;
        return *this;
    }

    constexpr bool operator==(const Radian &rhv) const noexcept
    {
        return _value == rhv._value;
    }

    constexpr bool operator!=(const Radian &rhv) const noexcept
    {
        return !(*this == rhv);
    }

    constexpr bool operator<(const Radian &rhv) const noexcept
    {
        return _value < rhv._value;
    }

    constexpr bool operator>=(const Radian &rhv) const noexcept
    {
        return !(*this < rhv);
    }

    constexpr bool operator>(const Radian &rhv) const noexcept
    {
        return _value > rhv._value;
    }

    constexpr bool operator<=(const Radian &rhv) const noexcept
    {
        return !(*this > rhv);
    }

    constexpr bool operator<(const Degree &rhv) const noexcept;
    constexpr bool operator>=(const Degree &rhv) const noexcept;
    constexpr bool operator>(const Degree &rhv) const noexcept;
    constexpr bool operator<=(const Degree &rhv) const noexcept;

    Radian normalize() const noexcept
    {
        return std::fmod(_value, 2 * M_PI);
    }

    friend std::ostream &operator<<(std::ostream &os, const Radian &rhv);

  private:
    double _value = 0.f;
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
    constexpr Degree(double value) : _value(value)
    {
    }

    constexpr Degree(Radian value) : _value(value.value() * 180 / M_PI)
    {
    }

    constexpr double value() const noexcept
    {
        return _value;
    }

    constexpr Radian normalize() const noexcept
    {
        return std::fmod(_value, 360);
    }

    constexpr bool operator==(const Degree &rhv) const noexcept
    {
        return _value == rhv._value;
    }

    constexpr bool operator!=(const Degree &rhv) const noexcept
    {
        return !(*this == rhv);
    }

  private:
    double _value = 0.f;
};

inline constexpr Radian::Radian(Degree value) : _value(value.value() * M_PI / 180.0)
{
}

constexpr bool Radian::operator<(const Degree &rhv) const noexcept
{
    return _value < Radian(rhv)._value;
}

constexpr bool Radian::operator>=(const Degree &rhv) const noexcept
{
    return !(*this < rhv);
}

constexpr bool Radian::operator>(const Degree &rhv) const noexcept
{
    return _value > Radian(rhv)._value;
}

constexpr bool Radian::operator<=(const Degree &rhv) const noexcept
{
    return !(*this > rhv);
}

class Kilometer;

/**
 * Meter
 * To store value in meters
 */
class Meter
{
  public:
    constexpr Meter(double value) : _value(value)
    {
    }

    constexpr Meter(Kilometer value);

    constexpr double value() const noexcept
    {
        return _value;
    }

    constexpr friend Meter operator-(Meter lhv, const Meter &rhv)
    {
        return lhv._value - rhv._value;
    }

    constexpr friend Meter operator-(Meter lhv, double rhv)
    {
        return lhv._value - rhv;
    }

    constexpr Meter &operator-=(const Meter &rhv)
    {
        _value -= rhv._value;
        return *this;
    }

    constexpr friend Meter operator+(Meter lhv, const Meter &rhv)
    {
        return lhv._value + rhv._value;
    }

    constexpr friend Meter operator+(Meter lhv, double rhv)
    {
        return lhv._value + rhv;
    }

    constexpr Meter &operator+=(const Meter &rhv)
    {
        _value += rhv._value;
        return *this;
    }

    constexpr friend Meter operator*(Meter lhv, double rhv)
    {
        return lhv._value * rhv;
    }

    constexpr Meter &operator*=(const Meter &rhv)
    {
        _value *= rhv._value;

        return *this;
    }

    constexpr friend Meter operator*(Meter lhv, const Meter &rhv)
    {
        lhv *= rhv;
        return lhv;
    }

    constexpr Meter &operator/=(const Meter &rhv)
    {
        _value /= rhv._value;

        return *this;
    }

    constexpr friend Meter operator/(Meter lhv, const Meter &rhv)
    {
        lhv /= rhv;
        return lhv;
    }

    constexpr bool operator==(const Meter &rhv) const noexcept
    {
        return _value == rhv._value;
    }

    constexpr bool operator!=(const Meter &rhv) const noexcept
    {
        return !(*this == rhv);
    }

    constexpr bool operator<=(const Meter &rhv) const noexcept
    {
        return _value <= rhv._value;
    }

    constexpr bool operator>(const Meter &rhv) const noexcept
    {
        return !(*this <= rhv);
    }

    constexpr bool operator>=(const Meter &rhv) const noexcept
    {
        return _value >= rhv._value;
    }

    constexpr bool operator<(const Meter &rhv) const noexcept
    {
        return !(*this >= rhv);
    }

    constexpr friend double operator*(const double &lhv, const Meter &rhv)
    {
        return lhv * rhv._value;
    }

    friend std::ostream &operator<<(std::ostream &os, const Meter &rhv)
    {
        return os << rhv._value;
    }

  private:
    double _value = 0.f;
};

/**
 * Kilometer
 * To store value in km
 */
class Kilometer
{
  public:
    constexpr Kilometer(double value) : _value(value)
    {
    }

    constexpr Kilometer(Meter value) : _value(value.value() / 1000)
    {
    }

    constexpr double value() const noexcept
    {
        return _value;
    }

    constexpr bool operator==(const Kilometer &rhv) const noexcept
    {
        return _value == rhv._value;
    }

    constexpr bool operator!=(const Kilometer &rhv) const noexcept
    {
        return !(*this == rhv);
    }

  private:
    double _value = 0.f;
};

inline constexpr Meter::Meter(Kilometer value) : _value(value.value() * 1000)
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
    constexpr Pos()
    {
    }

    constexpr Pos(Radian latitude, Radian longitude, Meter altitude)
        : _latitude(latitude), _longitude(longitude), _altitude(altitude)
    {
    }

    Pos(std::shared_ptr<const ReachGoalAction::Goal> goal)
        : _latitude(Degree(goal->goal_lat)), _longitude(Degree(goal->goal_long)), _altitude(0.f)
    {
    }
    Pos(const sensor_msgs::msg::NavSatFix &msg)
        : _latitude(Degree(msg.latitude)), _longitude(Degree(msg.longitude)), _altitude(msg.altitude)
    {
    }
    constexpr Radian latitude() const
    {
        return _latitude;
    }

    constexpr Radian longitude() const
    {
        return _longitude;
    }

    constexpr Meter altitude() const
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
        _latitude = Degree(msg.latitude);
        _longitude = Degree(msg.longitude);
        _altitude = msg.altitude;
        return *this;
    }

    constexpr bool operator==(const Pos &rhv) const noexcept
    {
        return std::tie(_latitude, _longitude, _altitude) == std::tie(rhv._latitude, rhv._longitude, rhv._altitude);
    }

    constexpr bool operator!=(const Pos &rhv) const noexcept
    {
        return !(*this == rhv);
    }

    friend std::ostream &operator<<(std::ostream &os, const Pos &rhv);
    friend std::ostream &operator<<(std::ostream &os, const Optional<Pos> &rhv);

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

using Vector3D = Cartesian;

// dot product of vectors
inline double operator*(const Vector3D &lhv, const Vector3D &rhv)
{
    return (lhv.x * rhv.x + lhv.y * rhv.y + lhv.z * rhv.z).value();
}

// cross product of vectors
inline Vector3D operator^(const Vector3D &lhv, const Vector3D &rhv)
{
    const auto a = lhv.y * rhv.z - rhv.y * lhv.z;
    const auto b = lhv.x * rhv.z - rhv.x * lhv.z;
    const auto c = lhv.x * rhv.y - rhv.x * lhv.y;
    return {a, -1 * b, c};
}

using Point = Cartesian;

/**
 * Plane
 * Represents the plane equation as a * x + b * y + c * z + d = 0
 */
struct Plane
{
    Meter a = 0.f;
    Meter b = 0.f;
    Meter c = 0.f;
    Meter d = 0.f;

    Plane(Meter ia, Meter ib, Meter ic, Meter id) : a(ia), b(ib), c(ic), d(id)
    {
    }
    Vector3D get_normal() const noexcept
    {
        return {a, b, c};
    }

    bool operator==(const Plane &rhv) const noexcept
    {
        return std::tie(a, b, c, d) == std::tie(rhv.a, rhv.b, rhv.c, rhv.d);
    }

    bool operator!=(const Plane &rhv) const noexcept
    {
        return !(*this == rhv);
    }
};

inline std::ostream &operator<<(std::ostream &os, const Plane &rhv)
{
    return os << "{ a: " << rhv.a << " b: " << rhv.b << " c: " << rhv.c << " d: " << rhv.d << "}";
}

inline double operator-(builtin_interfaces::msg::Time lhv, const builtin_interfaces::msg::Time &rhv)
{
    const auto passed_time = lhv.sec - rhv.sec + (static_cast<double>(lhv.nanosec) - rhv.nanosec) / 1'000'000'000;

    return passed_time;
}

struct Nullopt_t
{
    constexpr explicit Nullopt_t(int)
    {
    }
};

inline constexpr Nullopt_t Nullopt{0};

class BadOptionalAccess : public std::exception
{
  public:
    const char *what() const noexcept override
    {
        return "Bad Optional Access";
    }
};

/**
 * Optional
 * Similar to the std::optional from c++17 standard
 */
template <typename T> class Optional
{
  public:
    constexpr Optional() : _has(false), _value()
    {
    }
    constexpr Optional(Nullopt_t) : _has(false), _value()
    {
    }
    constexpr Optional(const T &value) : _has(true), _value(value)
    {
    }
    constexpr Optional(T &&value) : _has(true), _value(std::move(value))
    {
    }
    constexpr Optional(const Optional<T> &value) : _has(value._has), _value(value._value)
    {
    }
    constexpr Optional(Optional<T> &&value) : _has(value._has), _value(std::move(value._value))
    {
        value._has = false;
    }

    constexpr T &operator*()
    {
        return _value;
    }

    constexpr const T &operator*() const
    {
        return _value;
    }

    constexpr T *operator->()
    {
        return &_value;
    }

    constexpr const T *operator->() const
    {
        return &_value;
    }

    constexpr bool has_value() const noexcept
    {
        return _has;
    }

    constexpr T &value()
    {
        if (not _has)
        {
            throw BadOptionalAccess();
        }

        return _value;
    }
    constexpr const T &value() const
    {
        if (not _has)
        {
            throw BadOptionalAccess();
        }

        return _value;
    }

    Optional &operator=(const T &rhv)
    {
        _value = rhv;
        _has = true;
        return *this;
    }

    constexpr Optional &operator=(const Optional &rhv)
    {
        _value = rhv._value;
        _has = rhv._has;
        return *this;
    }

    void reset()
    {
        if (not _has)
        {
            return;
        }

        _has = false;

        if constexpr (not std::is_trivially_destructible_v<T> and std::is_nothrow_destructible_v<T>)
        {
            _value.~T();
        }
    }

  private:
    bool _has = false;
    T _value;
};

template<typename T>
inline std::ostream &operator<<(std::ostream &os, const Optional<T> &rhv)
{
    if (not rhv.has_value())
    {
        return os << "{ No data }";
    }

    return os << *rhv;
}