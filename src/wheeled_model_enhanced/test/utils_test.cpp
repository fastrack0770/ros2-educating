#include <gtest/gtest.h>

#include "wheeled_model_enhanced/utils.hpp"

TEST(utils, abs)
{
    EXPECT_FLOAT_EQ(6.020797289, utils::abs(Vector3D(3.2, -5.1, 0)));
}

TEST(utils, earth_radius_at)
{
    EXPECT_FLOAT_EQ(6378137.0, utils::earth_radius_at(Pos(Degree(0), Degree(12), 0)));
    // must be 6356752.314, but is not
    EXPECT_FLOAT_EQ(6360159, utils::earth_radius_at(Pos(Degree(66.564056), Degree(12), 122)));
}

TEST(utils, earth_radius_at_2)
{
    EXPECT_FLOAT_EQ(6378137.0, utils::earth_radius_at_2(Pos(Degree(0), Degree(12), 0)));
    // must be 6356752.314, but is not
    EXPECT_FLOAT_EQ(6360135, utils::earth_radius_at_2(Pos(Degree(66.564056), Degree(12), 122)));
}

TEST(utils, distance)
{
    // distance must be 376099.61 according to https://gps-coordinates.org/distance-between-coordinates.php
    // difference is not high, so will try to use it
    EXPECT_FLOAT_EQ(376094.41, utils::distance(Pos(Degree(35.652832), Degree(139.839478), 0),   // tokyo
                                                         Pos(Degree(35.011665), Degree(135.768326), 0)).value()); // kyoto
}

TEST(utils, get_angle_to_waypoint)
{
    using namespace utils;
    // robot looks north, angle to turn is 0 degrees
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(15, 0, 0);
        sensor_msgs::msg::Imu imu;
        imu.orientation.w = 1;
        imu.orientation.x = 0;
        imu.orientation.y = 0;
        imu.orientation.z = 0;

        EXPECT_FLOAT_EQ(0, to_deg(get_euler_z_angle(imu)));
        EXPECT_FLOAT_EQ(0, to_deg(get_angle_to_waypoint(robot, waypoint, imu)));
    }
    // robot looks north, angle to turn is 90 degrees
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(0, 3, 0);
        sensor_msgs::msg::Imu imu;
        imu.orientation.w = 1;
        imu.orientation.x = 0;
        imu.orientation.y = 0;
        imu.orientation.z = 0;

        EXPECT_FLOAT_EQ(0, to_deg(get_euler_z_angle(imu)));
        EXPECT_FLOAT_EQ(90, to_deg(get_angle_to_waypoint(robot, waypoint, imu)));
    }
}

TEST(utils, get_plane)
{
    using namespace utils;
    {
        const Point point(-1, 2, -3);
        const Vector3D lhv(4, 3, 2);
        const Vector3D rhv(-5, 7, 1);

        EXPECT_EQ(Plane(-11, 14, 43, 146), get_plane(point, lhv, rhv));
    }
    {
        const Point point(0, 0, 0);
        const Vector3D lhv(100, 0, 0);
        const Vector3D rhv(0, 3, 0);

        EXPECT_EQ(Plane(0, 0, 300, 0), get_plane(point, lhv, rhv));
    }
    {
        const Point point(0, 0, 0);
        const Vector3D lhv(100, 0, 0);
        const Vector3D rhv(0, -3, 0);

        EXPECT_EQ(Plane(0, 0, -300, 0), get_plane(point, lhv, rhv));
    }
}

TEST(utils, normalize)
{
    using namespace utils;
    
    EXPECT_EQ(Vector3D(0, 0, 1), normalize(Vector3D(0, 0, 300)));
}

TEST(utils, get_angle_to_waypoint_signed)
{
    using namespace utils;

    // robot looks north, angle to turn is 0 degrees
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(15, 0, 0);

        EXPECT_FLOAT_EQ(0, to_deg(get_angle_to_waypoint_signed(robot, waypoint, Radian(0)).value()));
    }
    // robot looks north, angle to turn is 0 degrees
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(-15, 0, 0);

        EXPECT_FLOAT_EQ(180, to_deg(get_angle_to_waypoint_signed(robot, waypoint, Radian(0)).value()));
    }
    // robot looks north, angle to turn is 90 degrees
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(0, 0, 3);

        EXPECT_FLOAT_EQ(90, to_deg(get_angle_to_waypoint_signed(robot, waypoint, Radian(0)).value()));
    }
    // robot looks north, angle to turn is -90
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(0, 0, -3);
        
        EXPECT_FLOAT_EQ(-90, to_deg(get_angle_to_waypoint_signed(robot, waypoint, Radian(0)).value()));
    }
    // north to the right of the robot by 45 degree, angle to turn is -45
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(0, 0, 3);
        
        EXPECT_FLOAT_EQ(135, to_deg(get_angle_to_waypoint_signed(robot, waypoint, Degree(45)).value()));
    }
    // north to the right of the robot by 45 degree, angle to turn is 135
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(0, 0, -3);
        
        EXPECT_FLOAT_EQ(-45, to_deg(get_angle_to_waypoint_signed(robot, waypoint, Degree(45)).value()));
    }
    // north to the left of the robot by 45 degree, angle to turn is -135
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(0, 0, 3);
        
        EXPECT_FLOAT_EQ(45, to_deg(get_angle_to_waypoint_signed(robot, waypoint, Degree(-45)).value()));
    }
    // north to the left of the robot by 45 degree, angle to turn is 45
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(0, 0, -3);
        
        EXPECT_FLOAT_EQ(-135, to_deg(get_angle_to_waypoint_signed(robot, waypoint, Degree(-45)).value()));
    }
    // waypoint to the left
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(-9.4050172, 0.68499405, -0.20115671); // { x: -9.4050172 y: 0.68499405 z: -0.20115671}
        const Radian robot_azimuth(1.5708);

        EXPECT_FLOAT_EQ(-88.774521, to_deg(get_angle_to_waypoint_signed(robot, waypoint,robot_azimuth).value()));
    }
    // waypoint to the right
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(9.4050172, 0.68499405, -0.20115671);
        const Radian robot_azimuth(1.5708);

        EXPECT_FLOAT_EQ(88.77494, to_deg(get_angle_to_waypoint_signed(robot, waypoint,robot_azimuth).value()));
    }
    // angle to waypoint is 135
    {
        const Cartesian robot(0, 0, 0);
        const Cartesian waypoint(9.6874266, 0.68499405, 9.9239567);
        const Radian robot_azimuth(1.5708);

        EXPECT_FLOAT_EQ(135.69121, to_deg(get_angle_to_waypoint_signed(robot, waypoint,robot_azimuth).value()));
    }
}

TEST(utils, get_angle_between_vectors_signed)
{
    using namespace utils;

    const Cartesian robot(0, 0, 0);
    const Cartesian waypoint(9.6874266, 0.68499405, 9.9239567);
    const auto wr_vec = make_vector(robot, waypoint); // Vector from robot to waypoint
    const auto rn_vec = make_vector(robot, Cartesian(robot.x + 100, robot.y, robot.z));

    auto rn_wr_angle = get_angle_between_vectors_signed(rn_vec, wr_vec);
    EXPECT_FLOAT_EQ(45.691002, to_deg(rn_wr_angle));
}

TEST(utils, get_speed)
{
    {
        const auto [velocity_to_set, s_ac] = utils::get_speed(1, 2, -0.433544);
        EXPECT_FLOAT_EQ(-0.65844059, velocity_to_set);
        EXPECT_FLOAT_EQ(0.216772, s_ac);
    }
}