#pragma once

#include "wheeled_model_enhanced/types/meter.hpp"
#include "wheeled_model_enhanced/types/vector3d.hpp"

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

    friend std::ostream &operator<<(std::ostream &os, const Plane &rhv)
    {
        return os << "{ a: " << rhv.a << " b: " << rhv.b << " c: " << rhv.c << " d: " << rhv.d << "}";
    }
};