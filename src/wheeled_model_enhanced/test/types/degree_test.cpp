#include <gtest/gtest.h>

#include "wheeled_model_enhanced/types/degree.hpp"

TEST(types, degree)
{
    // Radian(double value)
    {
        constexpr double value = -5.231;
        EXPECT_FLOAT_EQ(value, Degree(value).to_double());
    }
    // Radian(Degree value)
    {
        constexpr double value = 2.14675498;
        EXPECT_FLOAT_EQ(123, Degree(Radian(value)).to_double());
    }
}