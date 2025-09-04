#include "rclcpp/rclcpp.hpp"
#include "udemy_ros2_pkg/srv/odd_even_check.hpp"

using OddEvenCheck = udemy_ros2_pkg::srv::OddEvenCheck;

class OddEvenCheckServiceNode : public rclcpp::Node
{
public:
    OddEvenCheckServiceNode() : Node("odd_even_check_service_node")
    {
        const auto check_odd_even = [](const OddEvenCheck::Request::SharedPtr request,
                                       const OddEvenCheck::Response::SharedPtr response)
        {
            response->decision = request->number % 2 == 0 ? "Even" : "Odd";
        };
        _service_server = create_service<OddEvenCheck>(
            "odd_even_check",
            check_odd_even);
    }

private:
    rclcpp::Service<OddEvenCheck>::SharedPtr _service_server;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OddEvenCheckServiceNode>());
    rclcpp::shutdown();
}