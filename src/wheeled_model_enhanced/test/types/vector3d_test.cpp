#include <gtest/gtest.h>

#include "wheeled_model_enhanced/types/vector3d.hpp"

TEST(types, vector3d)
{
    // double operator*(const Vector3D &lhv, const Vector3D &rhv)
    EXPECT_FLOAT_EQ(0, Vector3D(3, 1, -4) * Vector3D(8, -8, 4));

    // Vector3D operator^(const Vector3D &lhv, const Vector3D &rhv)
    EXPECT_EQ(Vector3D(-5, 10, 0), Vector3D(4, 2, -3) ^ Vector3D(2, 1, -4));
}
