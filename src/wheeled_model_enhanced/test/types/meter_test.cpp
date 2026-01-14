#include <gtest/gtest.h>

#include "wheeled_model_enhanced/types/meter.hpp"

TEST(types, meter)
{
    // Meter(double value)
    {
        constexpr double value = -5.231;
        EXPECT_FLOAT_EQ(value, Meter(value).to_double());
    }
    // Meter(Kilometer value)
    {
        constexpr double value = -5.231;
        EXPECT_FLOAT_EQ(-5231, Meter(Kilometer(value)).to_double());
    }
    // Meter &operator-(const Meter &rhv)
    {
        EXPECT_FLOAT_EQ(-0.25, (Meter(-0.125) - Meter(0.125)).to_double());
    }
    // Meter &operator-(double rhv)
    {
        EXPECT_FLOAT_EQ(-0.25, (Meter(-0.125) - 0.125).to_double());
    }
    // Meter &operator*(double rhv)
    {
        EXPECT_FLOAT_EQ(-0.25, (Meter(-0.125) * 2.f).to_double());
    }
    // Meter &operator-=(const Meter &rhv)
    {
        EXPECT_FLOAT_EQ(-0.25, (Meter(-0.125) -= 0.125).to_double());
    }
    // Meter &operator+(const Meter &rhv)
    {
        EXPECT_FLOAT_EQ(0, (Meter(-0.125) + Meter(0.125)).to_double());
    }
    // Meter &operator+(double rhv)
    {
        EXPECT_FLOAT_EQ(0, (Meter(-0.125) + 0.125).to_double());
    }
    // Meter &operator+=(const Meter &rhv)
    {
        EXPECT_FLOAT_EQ(0, (Meter(-0.125) += 0.125).to_double());
    }
    // double operator*(const double & lhv, const Meter & rhv)
    {
        EXPECT_FLOAT_EQ(-0.25, 2.f * Meter(-0.125));
    }
}