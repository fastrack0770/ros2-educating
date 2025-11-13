#pragma once

#include <iostream>

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

#include "wheeled_model_enhanced/types/kilometer.hpp"

constexpr Meter::Meter(Kilometer value) : _value(value.value() * 1000)
{
}