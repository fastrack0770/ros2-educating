#include <gtest/gtest.h>

#include "wheeled_model_enhanced/types/pos.hpp"

TEST(types, radian)
{
    // Radian(double value)
    {
        constexpr double value = -5.231;
        EXPECT_FLOAT_EQ(value, Radian(value).to_double());
    }
    // Radian(Degree value)
    {
        constexpr double value = 123;
        EXPECT_FLOAT_EQ(2.14675498, Radian(Degree(value)).to_double());
    }
    // Radian normalize() const noexcept
    {
        constexpr double value = 7.783185307179586;
        EXPECT_FLOAT_EQ(1.5, Radian(value).normalize().to_double());
    }
}

