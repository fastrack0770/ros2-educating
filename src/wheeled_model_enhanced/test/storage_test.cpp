#include <gtest/gtest.h>

#include "wheeled_model_enhanced/storage.hpp"

TEST(storage, just_created)
{
    Storage storage;

    EXPECT_FALSE(storage.robot_gps_pos().has_value());
    EXPECT_FALSE(storage.robot_topo_pos().has_value());
    EXPECT_FALSE(storage.robot_related_pos().has_value());
    EXPECT_FALSE(storage.waypoint_gps_pos().has_value());
    EXPECT_FALSE(storage.waypoint_topo_pos().has_value());
    EXPECT_FALSE(storage.waypoint_related_pos().has_value());
    EXPECT_FLOAT_EQ(0, storage.distance_to_waypoint_gps().to_double());
    EXPECT_FLOAT_EQ(0, storage.distance_to_waypoint_related().to_double());
    EXPECT_FALSE(storage.angular_speed().has_value());
    EXPECT_FALSE(storage.has_angular_speed().has_value());
    EXPECT_FALSE(storage.linear_speed().has_value());
    EXPECT_FALSE(storage.has_linear_speed().has_value());
    EXPECT_FLOAT_EQ(0, storage.angular_acceleration());
    EXPECT_FALSE(storage.linear_acceleration().has_value());
    EXPECT_FLOAT_EQ(0, storage.angle_to_waypoint().to_double());
    EXPECT_FALSE(storage.robot_azimuth().has_value());
    EXPECT_FLOAT_EQ(0, storage.robot_imu_twist().to_double());
    EXPECT_FLOAT_EQ(0, storage.robot_length().to_double());
}

TEST(storage, got_robot_pos_first)
{
    Storage storage;

    storage.set_robot_imu_twist(1.5707963267948966);
    storage.set_robot_length(1.5);

    {
        sensor_msgs::msg::NavSatFix msg;
        msg.latitude = utils::to_deg(-0.40119337);
        msg.longitude = utils::to_deg(-0.75402585);
        msg.altitude = 0.32500458;

        storage.set_robot_pos(msg);
    }

    {
        EXPECT_FLOAT_EQ(-0.40119337, storage.robot_gps_pos()->latitude().to_double());
        EXPECT_FLOAT_EQ(-0.75402585, storage.robot_gps_pos()->longitude().to_double());
        EXPECT_FLOAT_EQ(0.32500458, storage.robot_gps_pos()->altitude().to_double());
    }
    {
        EXPECT_FLOAT_EQ(188792.47, storage.robot_topo_pos()->x.to_double());
        EXPECT_FLOAT_EQ(6338220.2, storage.robot_topo_pos()->y.to_double());
        EXPECT_FLOAT_EQ(0, storage.robot_topo_pos()->z.to_double());
    }
    EXPECT_EQ(Cartesian(0, 0, 0), *storage.robot_related_pos());
    EXPECT_FALSE(storage.waypoint_gps_pos().has_value());
    EXPECT_FALSE(storage.waypoint_topo_pos().has_value());
    EXPECT_FALSE(storage.waypoint_related_pos().has_value());
    EXPECT_FLOAT_EQ(0, storage.distance_to_waypoint_gps().to_double());
    EXPECT_FLOAT_EQ(0, storage.distance_to_waypoint_related().to_double());
    EXPECT_FALSE(storage.angular_speed().has_value());
    EXPECT_FALSE(storage.has_angular_speed().has_value());
    EXPECT_FALSE(storage.linear_speed().has_value());
    EXPECT_FALSE(storage.has_linear_speed().has_value());
    EXPECT_FLOAT_EQ(0, storage.angular_acceleration());
    EXPECT_FALSE(storage.linear_acceleration().has_value());
    EXPECT_FLOAT_EQ(0, storage.angle_to_waypoint().to_double());
    EXPECT_FALSE(storage.robot_azimuth().has_value());
    EXPECT_FLOAT_EQ(1.5707964, storage.robot_imu_twist().to_double());
    EXPECT_FLOAT_EQ(1.5, storage.robot_length().to_double());

    // got waypoint position
    {
        sensor_msgs::msg::NavSatFix msg;
        msg.latitude = utils::to_deg(-0.40119337);
        msg.longitude = utils::to_deg(-0.75402419);
        msg.altitude = 1.010006;

        storage.set_waypoint_pos(msg);
    }

    {
        EXPECT_FLOAT_EQ(-0.40119337, storage.robot_gps_pos()->latitude().to_double());
        EXPECT_FLOAT_EQ(-0.75402585, storage.robot_gps_pos()->longitude().to_double());
        EXPECT_FLOAT_EQ(0.32500458, storage.robot_gps_pos()->altitude().to_double());
    }
    {
        EXPECT_FLOAT_EQ(188792.47, storage.robot_topo_pos()->x.to_double());
        EXPECT_FLOAT_EQ(6338220.2, storage.robot_topo_pos()->y.to_double());
        EXPECT_FLOAT_EQ(0, storage.robot_topo_pos()->z.to_double());
    }
    EXPECT_EQ(Cartesian(0, 0, 0), *storage.robot_related_pos());
    {
        EXPECT_FLOAT_EQ(-0.40119337, storage.waypoint_gps_pos()->latitude().to_double());
        EXPECT_FLOAT_EQ(-0.75402419, storage.waypoint_gps_pos()->longitude().to_double());
        EXPECT_FLOAT_EQ(1.010006, storage.waypoint_gps_pos()->altitude().to_double());
    }
    {
        EXPECT_FLOAT_EQ(188792.47, storage.waypoint_topo_pos()->x.to_double());
        EXPECT_FLOAT_EQ(6338220.8, storage.waypoint_topo_pos()->y.to_double());
        EXPECT_FLOAT_EQ(9.8083839, storage.waypoint_topo_pos()->z.to_double());
    }
    {
        EXPECT_FLOAT_EQ(-3.1790696e-06, storage.waypoint_related_pos()->x.to_double());
        EXPECT_FLOAT_EQ(0.68499386, storage.waypoint_related_pos()->y.to_double());
        EXPECT_FLOAT_EQ(9.8083839, storage.waypoint_related_pos()->z.to_double());
    }
    EXPECT_FLOAT_EQ(8.2420483, storage.distance_to_waypoint_gps().to_double());
    EXPECT_FLOAT_EQ(8.3322735, storage.distance_to_waypoint_related().to_double());
    EXPECT_FALSE(storage.angular_speed().has_value());
    EXPECT_FALSE(storage.has_angular_speed().has_value());
    EXPECT_FALSE(storage.linear_speed().has_value());
    EXPECT_FALSE(storage.has_linear_speed().has_value());
    EXPECT_FLOAT_EQ(0, storage.angular_acceleration());
    EXPECT_FALSE(storage.linear_acceleration().has_value());
    EXPECT_FLOAT_EQ(0, storage.angle_to_waypoint().to_double()); // 0 due the storage didn't received imu - TODO make an Optional 
    EXPECT_FALSE(storage.robot_azimuth().has_value()); // TODO check azimuth in tests
    EXPECT_FLOAT_EQ(1.5707964, storage.robot_imu_twist().to_double());
    EXPECT_FLOAT_EQ(1.5, storage.robot_length().to_double());

    // got imu
    {
        sensor_msgs::msg::Imu msg;
        msg.orientation.set__w(1);
        msg.orientation.set__x(0);
        msg.orientation.set__y(0);
        msg.orientation.set__z(0);

        storage.set_imu(msg);
    }

    EXPECT_FLOAT_EQ(-3.1415926, storage.angle_to_waypoint().to_double());
    EXPECT_TRUE(storage.robot_azimuth().has_value());
    EXPECT_FLOAT_EQ(M_PI / 2, storage.robot_azimuth().value().to_double());
}

TEST(storage, got_waypoint_pos_first)
{
    Storage storage;

    storage.set_robot_imu_twist(1.5707963267948966);
    storage.set_robot_length(1.5);

    {
        sensor_msgs::msg::NavSatFix msg;
        msg.latitude = utils::to_deg(-0.40119337);
        msg.longitude = utils::to_deg(-0.75402419);
        msg.altitude = 1.010006;

        storage.set_waypoint_pos(msg);
    }

    EXPECT_FALSE(storage.robot_gps_pos().has_value());
    EXPECT_FALSE(storage.robot_topo_pos().has_value());
    EXPECT_FALSE(storage.robot_related_pos().has_value());
    {
        EXPECT_FLOAT_EQ(-0.40119337, storage.waypoint_gps_pos()->latitude().to_double());
        EXPECT_FLOAT_EQ(-0.75402419, storage.waypoint_gps_pos()->longitude().to_double());
        EXPECT_FLOAT_EQ(1.010006, storage.waypoint_gps_pos()->altitude().to_double());
    }
    EXPECT_FALSE(storage.waypoint_topo_pos().has_value());
    EXPECT_FALSE(storage.waypoint_related_pos().has_value());
    EXPECT_FLOAT_EQ(0, storage.distance_to_waypoint_gps().to_double());
    EXPECT_FLOAT_EQ(0, storage.distance_to_waypoint_related().to_double());
    EXPECT_FALSE(storage.angular_speed().has_value());
    EXPECT_FALSE(storage.has_angular_speed().has_value());
    EXPECT_FALSE(storage.linear_speed().has_value());
    EXPECT_FALSE(storage.has_linear_speed().has_value());
    EXPECT_FLOAT_EQ(0, storage.angular_acceleration());
    EXPECT_FALSE(storage.linear_acceleration().has_value());
    EXPECT_FLOAT_EQ(0, storage.angle_to_waypoint().to_double());
    EXPECT_FALSE(storage.robot_azimuth().has_value());
    EXPECT_FLOAT_EQ(1.5707964, storage.robot_imu_twist().to_double());
    EXPECT_FLOAT_EQ(1.5, storage.robot_length().to_double());

    // got robot position
    {
        sensor_msgs::msg::NavSatFix msg;
        msg.latitude = utils::to_deg(-0.40119337);
        msg.longitude = utils::to_deg(-0.75402585);
        msg.altitude = 0.32500458;

        storage.set_robot_pos(msg);
    }

    {
        EXPECT_FLOAT_EQ(-0.40119337, storage.robot_gps_pos()->latitude().to_double());
        EXPECT_FLOAT_EQ(-0.75402585, storage.robot_gps_pos()->longitude().to_double());
        EXPECT_FLOAT_EQ(0.32500458, storage.robot_gps_pos()->altitude().to_double());
    }
    {
        EXPECT_FLOAT_EQ(188792.47, storage.robot_topo_pos()->x.to_double());
        EXPECT_FLOAT_EQ(6338220.2, storage.robot_topo_pos()->y.to_double());
        EXPECT_FLOAT_EQ(0, storage.robot_topo_pos()->z.to_double());
    }
    EXPECT_EQ(Cartesian(0, 0, 0), *storage.robot_related_pos());
    {
        EXPECT_FLOAT_EQ(-0.40119337, storage.waypoint_gps_pos()->latitude().to_double());
        EXPECT_FLOAT_EQ(-0.75402419, storage.waypoint_gps_pos()->longitude().to_double());
        EXPECT_FLOAT_EQ(1.010006, storage.waypoint_gps_pos()->altitude().to_double());
    }
    {
        EXPECT_FLOAT_EQ(188792.47, storage.waypoint_topo_pos()->x.to_double());
        EXPECT_FLOAT_EQ(6338220.8, storage.waypoint_topo_pos()->y.to_double());
        EXPECT_FLOAT_EQ(9.8083839, storage.waypoint_topo_pos()->z.to_double());
    }
    {
        EXPECT_FLOAT_EQ(-3.1790696e-06, storage.waypoint_related_pos()->x.to_double());
        EXPECT_FLOAT_EQ(0.68499386, storage.waypoint_related_pos()->y.to_double());
        EXPECT_FLOAT_EQ(9.8083839, storage.waypoint_related_pos()->z.to_double());
    }
    EXPECT_FLOAT_EQ(8.2420483, storage.distance_to_waypoint_gps().to_double());
    EXPECT_FLOAT_EQ(8.3322735, storage.distance_to_waypoint_related().to_double());
    EXPECT_FALSE(storage.angular_speed().has_value());
    EXPECT_FALSE(storage.has_angular_speed().has_value());
    EXPECT_FALSE(storage.linear_speed().has_value());
    EXPECT_FALSE(storage.has_linear_speed().has_value());
    EXPECT_FLOAT_EQ(0, storage.angular_acceleration());
    EXPECT_FALSE(storage.linear_acceleration().has_value());
    EXPECT_FLOAT_EQ(0, storage.angle_to_waypoint().to_double()); // 0 due the storage didn't received imu
    EXPECT_FALSE(storage.robot_azimuth().has_value());
    EXPECT_FLOAT_EQ(1.5707964, storage.robot_imu_twist().to_double());
    EXPECT_FLOAT_EQ(1.5, storage.robot_length().to_double());

    // got imu
    {
        sensor_msgs::msg::Imu msg;
        msg.orientation.set__w(1);
        msg.orientation.set__x(0);
        msg.orientation.set__y(0);
        msg.orientation.set__z(0);

        storage.set_imu(msg);
    }

    EXPECT_FLOAT_EQ(-3.1415926, storage.angle_to_waypoint().to_double());
}

TEST(storage, angular_linear_speed)
{
    Storage storage;

    EXPECT_FALSE(storage.angular_speed().has_value());
    EXPECT_FALSE(storage.has_angular_speed().has_value());
    EXPECT_FALSE(storage.linear_speed().has_value());
    EXPECT_FALSE(storage.has_linear_speed().has_value());

    // got odometry
    {
        nav_msgs::msg::Odometry msg;
        msg.twist.twist.angular.z = 5;
        msg.twist.twist.linear.x = 7;

        storage.set_odometry(msg);
    }

    EXPECT_FLOAT_EQ(5, storage.angular_speed().value());
    EXPECT_TRUE(storage.has_angular_speed().value());
    EXPECT_FLOAT_EQ(7, storage.linear_speed().value());
    EXPECT_TRUE(storage.has_linear_speed().value());
}

TEST(storage, angular_linear_acceleration)
{
    Storage storage;

    EXPECT_FLOAT_EQ(0, storage.angular_acceleration());
    EXPECT_FALSE(storage.linear_acceleration().has_value());

    // set linear acceleration 
    {
        sensor_msgs::msg::Imu msg;

        msg.linear_acceleration.x = 10;

        storage.set_imu(msg);
    }
    // set angular acceleration
    {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp.set__sec(5);
        msg.twist.twist.angular.z = 10;

        storage.set_odometry(msg);
    }
    {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp.set__sec(6);
        msg.twist.twist.angular.z = 15;

        storage.set_odometry(msg);
    }

    EXPECT_FLOAT_EQ(10, storage.linear_acceleration().value());
    EXPECT_FLOAT_EQ(5, storage.angular_acceleration());
}