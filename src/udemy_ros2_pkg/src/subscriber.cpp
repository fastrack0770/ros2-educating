#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

/**
 * HelloWorldSubNode
 * 
 * Example of a subscriber node based on ros2 subscriber demo
 */
class HelloWorldSubNode : public rclcpp::Node
{
public:
    HelloWorldSubNode() : Node("hello_world_pub_node")
    {
        const auto callback = [this](const std_msgs::msg::String & msg)
        {
            RCLCPP_INFO(get_logger(), msg.data.c_str());
        };

        _subscription = create_subscription<std_msgs::msg::String>(
            "hello_world", 10, callback);
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloWorldSubNode>());
    rclcpp::shutdown();
    return 0;
}