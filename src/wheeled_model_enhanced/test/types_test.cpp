#include <gtest/gtest.h>

#include "wheeled_model_enhanced/types.hpp"

TEST(types, builtin_interfaces_msg_Time)
{
    const auto fill_time = [](uint32_t sec, uint32_t nanosec) {
        builtin_interfaces::msg::Time time;
        time.set__sec(sec);
        time.set__nanosec(nanosec);

        return time;
    };

    {
        const auto lhv = fill_time(2, 1);
        const auto rhv = fill_time(2, 1);

        EXPECT_FLOAT_EQ(0, lhv - rhv);
        EXPECT_FLOAT_EQ(0, lhv - lhv);
    }
    {
        const auto lhv = fill_time(2, 0);
        const auto rhv = fill_time(2, 1);

        EXPECT_FLOAT_EQ(-0.000000001, lhv - rhv);
        EXPECT_FLOAT_EQ(0.000000001, rhv - lhv);
    }
    {
        const auto lhv = fill_time(1761895672, 159938976);
        const auto rhv = fill_time(1761895671, 151234221);

        EXPECT_FLOAT_EQ(1.0087049007415771, lhv - rhv);
        EXPECT_FLOAT_EQ(-1.0087049007415771, rhv - lhv);
    }
}
