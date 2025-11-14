#include <gtest/gtest.h>

#include "wheeled_model_enhanced/types/meter.hpp"

TEST(types, meter)
{
    // Meter(double value)
    {
        constexpr double value = -5.231;
        EXPECT_FLOAT_EQ(value, Meter(value).value());
    }
    // Meter(Kilometer value)
    {
        constexpr double value = -5.231;
        EXPECT_FLOAT_EQ(-5231, Meter(Kilometer(value)).value());
    }
    // Meter &operator-(const Meter &rhv)
    {
        EXPECT_FLOAT_EQ(-0.25, (Meter(-0.125) - Meter(0.125)).value());
    }
    // Meter &operator-(double rhv)
    {
        EXPECT_FLOAT_EQ(-0.25, (Meter(-0.125) - 0.125).value());
    }
    // Meter &operator*(double rhv)
    {
        EXPECT_FLOAT_EQ(-0.25, (Meter(-0.125) * 2.f).value());
    }
    // Meter &operator-=(const Meter &rhv)
    {
        EXPECT_FLOAT_EQ(-0.25, (Meter(-0.125) -= 0.125).value());
    }
    // Meter &operator+(const Meter &rhv)
    {
        EXPECT_FLOAT_EQ(0, (Meter(-0.125) + Meter(0.125)).value());
    }
    // Meter &operator+(double rhv)
    {
        EXPECT_FLOAT_EQ(0, (Meter(-0.125) + 0.125).value());
    }
    // Meter &operator+=(const Meter &rhv)
    {
        EXPECT_FLOAT_EQ(0, (Meter(-0.125) += 0.125).value());
    }
    // double operator*(const double & lhv, const Meter & rhv)
    {
        EXPECT_FLOAT_EQ(-0.25, 2.f * Meter(-0.125));
    }
}