#include <gtest/gtest.h>

#include "wheeled_model_enhanced/types.hpp"

TEST(types, radian)
{
    // Radian(double value)
    {
        constexpr double value = -5.231;
        EXPECT_FLOAT_EQ(value, Radian(value).value());
    }
    // Radian(Degree value)
    {
        constexpr double value = 123;
        EXPECT_FLOAT_EQ(2.14675498, Radian(Degree(value)).value());
    }
    // Radian normalize() const noexcept
    {
        constexpr double value = 7.783185307179586;
        EXPECT_FLOAT_EQ(1.5, Radian(value).normalize().value());
    }
}

TEST(types, degree)
{
    // Radian(double value)
    {
        constexpr double value = -5.231;
        EXPECT_FLOAT_EQ(value, Degree(value).value());
    }
    // Radian(Degree value)
    {
        constexpr double value = 2.14675498;
        EXPECT_FLOAT_EQ(123, Degree(Radian(value)).value());
    }
}

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

TEST(types, pos)
{
    // Pos(const Pos &copy)
    {
        const Pos to_copy(0.5, 0.6, 0.7);
        EXPECT_EQ(to_copy, Pos(to_copy));
    }
    // Pos(std::shared_ptr<const ReachGoalAction::Goal> goal)
    {
        using ReachGoalAction = wheeled_model_enhanced::action::ReachGoal;

        const auto goal = std::make_shared<ReachGoalAction::Goal>();
        goal->goal_lat = 123.22;
        goal->goal_long = -23.2;

        EXPECT_EQ(Pos(Degree(123.22), Degree(-23.2), Meter(0)), Pos(goal)) << Pos(goal);
    }
    // pos getters and setters
    {
        Pos pos(2.5, -1.2, 0.7);
        EXPECT_FLOAT_EQ(2.5, pos.latitude().value()) << pos;
        EXPECT_FLOAT_EQ(-1.2, pos.longitude().value()) << pos;
        EXPECT_FLOAT_EQ(0.7, pos.altitude().value()) << pos;

        pos.setLatitude(0.5);
        pos.setLongitude(0.6);
        pos.setAltitude(0.7);

        EXPECT_FLOAT_EQ(0.5, pos.latitude().value());
        EXPECT_FLOAT_EQ(0.6, pos.longitude().value());
        EXPECT_FLOAT_EQ(0.7, pos.altitude().value());
    }
}

TEST(types, cartesian)
{
    // Cartesian operator-(Cartesian lhv, const Cartesian &rhv)
    {
        Cartesian lhv(5, 7, 9);
        Cartesian rhv(2, 3, 4);
        EXPECT_EQ(Cartesian(3, 4, 5), lhv - rhv);
        EXPECT_EQ(Cartesian(5, 7, 9), lhv) << lhv;
    }
    // Cartesian operator+(Cartesian lhv, const Cartesian &rhv)
    {
        Cartesian lhv(5, 7, 9);
        Cartesian rhv(2, 3, 4);
        EXPECT_EQ(Cartesian(7, 10, 13), lhv + rhv);
        EXPECT_EQ(Cartesian(5, 7, 9), lhv) << lhv;
    }
}

TEST(types, vector3d)
{
    // double operator*(const Vector3D &lhv, const Vector3D &rhv)
    EXPECT_FLOAT_EQ(0, Vector3D(3, 1, -4) * Vector3D(8, -8, 4));

    // Vector3D operator^(const Vector3D &lhv, const Vector3D &rhv)
    EXPECT_EQ(Vector3D(-5, 10, 0), Vector3D(4, 2, -3) ^ Vector3D(2, 1, -4));
}

TEST(types, plane)
{
    EXPECT_EQ(Vector3D(0, 0, 300), Plane(0, 0, 300, 0).get_normal());
}

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