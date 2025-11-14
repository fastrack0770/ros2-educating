#include <gtest/gtest.h>

#include "wheeled_model_enhanced/types/plane.hpp"

TEST(types, plane)
{
    EXPECT_EQ(Vector3D(0, 0, 300), Plane(0, 0, 300, 0).get_normal());
}