#include <gtest/gtest.h>

#include "wheeled_model_enhanced/types/pos.hpp"

TEST(types, pos)
{
    // constexpr Pos()
    {
        constexpr Pos pos;
        EXPECT_FLOAT_EQ(0, pos.latitude().to_double());
        EXPECT_FLOAT_EQ(0, pos.longitude().to_double());
        EXPECT_FLOAT_EQ(0, pos.altitude().to_double());
    }
    // constexpr Pos(Radian latitude, Radian longitude, Meter altitude)
    {
        constexpr Pos pos(5.1, 7.2, 9.3);
        EXPECT_FLOAT_EQ(5.1, pos.latitude().to_double());
        EXPECT_FLOAT_EQ(7.2, pos.longitude().to_double());
        EXPECT_FLOAT_EQ(9.3, pos.altitude().to_double());
    }
    // Pos(std::shared_ptr<const ReachGoalAction::Goal> goal)
    {
        using ReachGoalAction = wheeled_model_enhanced::action::ReachGoal;

        const auto goal = std::make_shared<ReachGoalAction::Goal>();
        goal->goal_lat = 123.22;
        goal->goal_long = -23.2;

        EXPECT_EQ(Pos(123.22, -23.2, Meter(0)), Pos(goal)) << Pos(goal);
    }
    // Pos &operator=(const sensor_msgs::msg::NavSatFix &msg)
    {
        Pos pos(1, 2, 3);
        sensor_msgs::msg::NavSatFix msg;
        msg.latitude = Degree(Radian(1.23)).to_double();
        msg.longitude = Degree(Radian(2.11)).to_double();
        msg.altitude = 122;

        pos = msg;

        EXPECT_FLOAT_EQ(1.23, pos.latitude().to_double());
        EXPECT_FLOAT_EQ(2.11, pos.longitude().to_double());
        EXPECT_FLOAT_EQ(122, pos.altitude().to_double());
    }
    // constexpr bool operator==(const Pos &rhv) const noexcept
    {
        constexpr Pos lhv(1, 2, 3);
        constexpr Pos rhv(1, 2, 3);

        EXPECT_TRUE(lhv == rhv);
    }
    {
        constexpr Pos lhv(1, 2, 3);
        constexpr Pos rhv(1, 2, 3.1);

        EXPECT_FALSE(lhv == rhv);
    }
    // pos getters and setters
    {
        Pos pos(2.5, -1.2, 0.7);
        EXPECT_FLOAT_EQ(2.5, pos.latitude().to_double()) << pos;
        EXPECT_FLOAT_EQ(-1.2, pos.longitude().to_double()) << pos;
        EXPECT_FLOAT_EQ(0.7, pos.altitude().to_double()) << pos;

        pos.setLatitude(0.5);
        pos.setLongitude(0.6);
        pos.setAltitude(0.7);

        EXPECT_FLOAT_EQ(0.5, pos.latitude().to_double());
        EXPECT_FLOAT_EQ(0.6, pos.longitude().to_double());
        EXPECT_FLOAT_EQ(0.7, pos.altitude().to_double());
    }
}
