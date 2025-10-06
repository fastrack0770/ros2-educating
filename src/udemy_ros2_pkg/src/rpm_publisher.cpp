#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>

using namespace std::chrono_literals;

static constexpr double RPM_DEFAULT_VALUE = 10;
/**
 * RpmPubNode
 * 
 * Created for an education purpose based on publisher node (publisher.cpp)
 */
class RpmPubNode : public rclcpp::Node
{
public:
    RpmPubNode() : Node("rpm_pub_node")
    {
        declare_parameter<double>("rpm_val", RPM_DEFAULT_VALUE);
        _publisher = create_publisher<std_msgs::msg::Float64>(
            "rpm", 10);

        const auto rpm_pub_callback = [this]()
        {
            auto msg = std_msgs::msg::Float64();
            const auto rpm_value = get_parameter("rpm_val");
            msg.data = rpm_value.as_double();

            _publisher->publish(msg);
        };
        _timer = create_wall_timer(std::chrono::seconds(1), rpm_pub_callback);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RpmPubNode>());
    rclcpp::shutdown();
    return 0;
}