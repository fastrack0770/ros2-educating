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
    EXPECT_FLOAT_EQ(0, storage.distance_to_waypoint_gps().value());
    EXPECT_FLOAT_EQ(0, storage.distance_to_waypoint_related().value());
    EXPECT_FLOAT_EQ(0, storage.angular_speed());
    EXPECT_EQ(false, storage.has_angular_speed());
    EXPECT_FLOAT_EQ(0, storage.linear_speed());
    EXPECT_EQ(false, storage.has_linear_speed());
    EXPECT_FLOAT_EQ(0, storage.angular_acceleration());
    EXPECT_FLOAT_EQ(0, storage.linear_acceleration());
    EXPECT_FLOAT_EQ(0, storage.angle_to_waypoint().value());
    EXPECT_FLOAT_EQ(0, storage.robot_azimuth().value());
    EXPECT_FLOAT_EQ(0, storage.robot_imu_twist().value());
    EXPECT_FLOAT_EQ(0, storage.robot_length().value());
}

TEST(storage, got_robot_pos_first)
{
    // WAYPOINT gps: { lat: -0.40119337 long: -0.75402419 alt: 1.2499784}, topoc: { x: -2301383 y: 4307061.9 z:
    // -4044940.3}, related: { x: -2301383 y: 4307061.9 z: -4044940.3}
    // ROBOT gps: { lat: -0.40119337 long: -0.75402585
    // alt: 0.4821136}, topoc: { x: 188792.47 y: 6338220.2 z: 4.6566129e-10}, related: { x: 0 y: 0 z: 0}

    Storage storage;
    {
        sensor_msgs::msg::NavSatFix msg;
        msg.latitude = utils::to_deg(-0.40119337);
        msg.longitude = utils::to_deg(-0.75402585);
        msg.altitude = 0.4821136;

        storage.set_robot_pos(msg);
    }

    {
        EXPECT_FLOAT_EQ(-0.40119337, storage.robot_gps_pos()->latitude().value());
        EXPECT_FLOAT_EQ(-0.75402585, storage.robot_gps_pos()->longitude().value());
        EXPECT_FLOAT_EQ(0.4821136, storage.robot_gps_pos()->altitude().value());
    }
    {
        EXPECT_FLOAT_EQ(188792.47, storage.robot_topo_pos()->x.value());
        EXPECT_FLOAT_EQ(6338220.2, storage.robot_topo_pos()->y.value());
        EXPECT_FLOAT_EQ(4.6566129e-10, storage.robot_topo_pos()->z.value());
    }
    EXPECT_EQ(Cartesian(0, 0, 0), *storage.robot_related_pos());
    EXPECT_FALSE(storage.waypoint_gps_pos().has_value());
    EXPECT_FALSE(storage.waypoint_topo_pos().has_value());
    EXPECT_FALSE(storage.waypoint_related_pos().has_value());
    EXPECT_FLOAT_EQ(0, storage.distance_to_waypoint_gps().value());
    EXPECT_FLOAT_EQ(0, storage.distance_to_waypoint_related().value());
    EXPECT_FLOAT_EQ(0, storage.angular_speed());
    EXPECT_EQ(false, storage.has_angular_speed());
    EXPECT_FLOAT_EQ(0, storage.linear_speed());
    EXPECT_EQ(false, storage.has_linear_speed());
    EXPECT_FLOAT_EQ(0, storage.angular_acceleration());
    EXPECT_FLOAT_EQ(0, storage.linear_acceleration());
    EXPECT_FLOAT_EQ(0, storage.angle_to_waypoint().value());
    EXPECT_FLOAT_EQ(0, storage.robot_azimuth().value());
    EXPECT_FLOAT_EQ(0, storage.robot_imu_twist().value());
    EXPECT_FLOAT_EQ(0, storage.robot_length().value());
}

TEST(storage, got_waypoint_pos_first)
{
    // WAYPOINT gps: { lat: -0.40119337 long: -0.75402419 alt: 1.2499784}, topoc: { x: -2301383 y: 4307061.9 z:
    // -4044940.3}, related: { x: -2301383 y: 4307061.9 z: -4044940.3}
    // ROBOT gps: { lat: -0.40119337 long: -0.75402585
    // alt: 0.4821136}, topoc: { x: 188792.47 y: 6338220.2 z: 4.6566129e-10}, related: { x: 0 y: 0 z: 0}

    Storage storage;
    {
        sensor_msgs::msg::NavSatFix msg;
        msg.latitude = utils::to_deg(-0.40119337);
        msg.longitude = utils::to_deg(-0.75402419);
        msg.altitude = 1.2499784;

        storage.set_waypoint_pos(msg);
    }

    EXPECT_FALSE(storage.robot_gps_pos().has_value());
    EXPECT_FALSE(storage.robot_topo_pos().has_value());
    EXPECT_FALSE(storage.robot_related_pos().has_value());
    {
        EXPECT_FLOAT_EQ(-0.40119337, storage.waypoint_gps_pos()->latitude().value());
        EXPECT_FLOAT_EQ(-0.75402419, storage.waypoint_gps_pos()->longitude().value());
        EXPECT_FLOAT_EQ(1.2499784, storage.waypoint_gps_pos()->altitude().value());
    }
    EXPECT_FALSE(storage.waypoint_topo_pos().has_value());
    EXPECT_FALSE(storage.waypoint_related_pos().has_value());
    EXPECT_FLOAT_EQ(0, storage.distance_to_waypoint_gps().value());
    EXPECT_FLOAT_EQ(0, storage.distance_to_waypoint_related().value());
    EXPECT_FLOAT_EQ(0, storage.angular_speed());
    EXPECT_EQ(false, storage.has_angular_speed());
    EXPECT_FLOAT_EQ(0, storage.linear_speed());
    EXPECT_EQ(false, storage.has_linear_speed());
    EXPECT_FLOAT_EQ(0, storage.angular_acceleration());
    EXPECT_FLOAT_EQ(0, storage.linear_acceleration());
    EXPECT_FLOAT_EQ(0, storage.angle_to_waypoint().value());
    EXPECT_FLOAT_EQ(0, storage.robot_azimuth().value());
    EXPECT_FLOAT_EQ(0, storage.robot_imu_twist().value());
    EXPECT_FLOAT_EQ(0, storage.robot_length().value());
}