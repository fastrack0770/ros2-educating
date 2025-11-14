#pragma once

#include "wheeled_model_enhanced/types/meter.hpp"

#include <tuple>

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

    friend std::ostream &operator<<(std::ostream &os, const Cartesian &rhv)
    {
        return os << "{ x: " << rhv.x << " y: " << rhv.y << " z: " << rhv.z << "}";
    }
};