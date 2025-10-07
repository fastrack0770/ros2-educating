#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;

/**
 * HelloWorldPubNode
 * 
 * Example of publisher based on ros2 demo publisher
 */
class HelloWorldPubNode : public rclcpp::Node
{
public:
    HelloWorldPubNode() : Node("hello_world_pub_node")
    {
        _publisher = create_publisher<std_msgs::msg::String>(
            "hello_world", 10);

        const auto hello_callback = [this]()
        {
            static auto counter = 1;
            auto msg = std_msgs::msg::String();
            msg.data = "Hello " + std::to_string(counter++);

            _publisher->publish(msg);
        };
        _timer = create_wall_timer(std::chrono::seconds(1), hello_callback);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloWorldPubNode>());
    rclcpp::shutdown();
    return 0;
}