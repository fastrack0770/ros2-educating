#pragma once

#include "wheeled_model_enhanced/types/cartesian.hpp"

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
