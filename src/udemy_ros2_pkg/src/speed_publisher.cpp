#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <iostream>

static constexpr double WHEEL_RADIUS_DEFAULT_VALUE = 0.15;

/**
 * SpeedPubNode
 * 
 * Created based on the publisher node (publisher.cpp) and subscriber node (subscriber.cpp).  
 * Waits for a rpm message from a "rpm" topic, calculates a speed and send it to a "speed" topic
 */
class SpeedPubNode : public rclcpp::Node
{
public:
    SpeedPubNode() : Node("speed_pub_node")
    {
        declare_parameter<double>("wheel_rad", WHEEL_RADIUS_DEFAULT_VALUE);
        const auto callback = [this](const std_msgs::msg::Float64 &rpm_msg)
        {
            const auto wheel_radius = get_parameter("wheel_rad").as_double();

            auto speed = 2 * wheel_radius * M_PI * rpm_msg.data / 60;
            RCLCPP_INFO(get_logger(), (std::string("rpm: ") + std::to_string(rpm_msg.data) + ", speed: " + std::to_string(speed)).c_str());

            auto speed_msg = std_msgs::msg::Float64();
            speed_msg.data = speed;

            _publisher->publish(speed_msg);
        };

        _subscription = create_subscription<std_msgs::msg::Float64>(
            "rpm", 10, callback);
        _publisher = create_publisher<std_msgs::msg::Float64>(
            "speed", 10);
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _subscription;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _publisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedPubNode>());
    rclcpp::shutdown();
    return 0;
}