#include <gtest/gtest.h>

#include "wheeled_model_enhanced/types/kilometer.hpp"

TEST(types, kilometer)
{
    // Kilometer(double value)
    {
        constexpr double value = -5.231;
        EXPECT_FLOAT_EQ(value, Kilometer(value).value());
    }
    // Kilometer(Meter value)
    {
        constexpr double value = -5.231;
        EXPECT_FLOAT_EQ(-0.005231, Kilometer(Meter(value)).value());
    }
}