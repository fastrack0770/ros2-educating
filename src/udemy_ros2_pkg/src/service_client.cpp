#include "rclcpp/rclcpp.hpp"
#include "udemy_ros2_pkg/srv/odd_even_check.hpp"

#include <iostream>

using OddEvenCheck = udemy_ros2_pkg::srv::OddEvenCheck;

/**
 * Service client example.
 * How does it work:
 * 1. Connect to server
 * 2. Wait for an input
 * 3. Send a message to server
 * 4. Wait for a response
 * 5. Output a result
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto service_client_node = rclcpp::Node::make_shared("odd_even_check_client_node");
    auto client = service_client_node->create_client<OddEvenCheck>("odd_even_check");

    auto request = std::make_shared<OddEvenCheck::Request>();

    std::cout << "Input a number" << std::endl;
    std::cin >> request->number;

    client->wait_for_service();
    auto result = client->async_send_request(request);
    const auto err = rclcpp::spin_until_future_complete(service_client_node, result);
    if (err == rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cout << "Number is: " << result.get()->decision << std::endl;
    }
    else
    {
        std::cout << "Error " << rclcpp::to_string(err) << std::endl;
    }

    rclcpp::shutdown();
}