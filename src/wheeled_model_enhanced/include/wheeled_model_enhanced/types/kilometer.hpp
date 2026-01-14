#pragma once

class Meter;

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

    constexpr Kilometer(const Meter &value);

    constexpr double to_double() const noexcept
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

#include "wheeled_model_enhanced/types/meter.hpp"

constexpr Kilometer::Kilometer(const Meter & value) : _value(value.to_double() / 1000)
{
}