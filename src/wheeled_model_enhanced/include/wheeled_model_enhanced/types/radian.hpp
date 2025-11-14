#pragma once

#include <cmath>
#include <iostream>

class Degree;

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

    constexpr Radian(const Degree &value);

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
    constexpr bool operator>=(const Degree &rhv) const noexcept
    {
        return !(*this < rhv);
    }

    constexpr bool operator>(const Degree &rhv) const noexcept;
    constexpr bool operator<=(const Degree &rhv) const noexcept
    {
        return !(*this > rhv);
    }

    constexpr Radian normalize() const noexcept
    {
        return std::fmod(_value, 2 * M_PI);
    }

    friend std::ostream &operator<<(std::ostream &os, const Radian &rhv)
    {
        return os << rhv._value;
    }

  private:
    double _value = 0.f;
};

#include "wheeled_model_enhanced/types/degree.hpp"

constexpr Radian::Radian(const Degree &value) : _value(value.value() * M_PI / 180.0)
{
}

constexpr bool Radian::operator<(const Degree &rhv) const noexcept
{
    return _value < Radian(rhv)._value;
}

constexpr bool Radian::operator>(const Degree &rhv) const noexcept
{
    return _value > Radian(rhv)._value;
}