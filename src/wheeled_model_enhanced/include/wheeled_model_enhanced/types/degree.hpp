#pragma once

#include <cmath>

class Radian;

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

    constexpr Degree(const Radian &value);

    constexpr double value() const noexcept
    {
        return _value;
    }

    constexpr Degree normalize() const noexcept
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

#include "wheeled_model_enhanced/types/radian.hpp"

constexpr Degree::Degree(const Radian &value) : _value(value.value() * 180 / M_PI)
{
}