#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "udemy_ros2_pkg/srv/turn.hpp"

#include <iostream>

using Turn = udemy_ros2_pkg::srv::Turn;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto service_client_node = rclcpp::Node::make_shared("turn_client_node");
    auto client = service_client_node->create_client<Turn>("turn");

    auto request = std::make_shared<Turn::Request>();

    std::cout << "Input an angle" << std::endl;
    std::cin >> request->angle;

    std::cout << "Waiting for service" << std::endl;
    client->wait_for_service();
    auto result = client->async_send_request(request);
    const auto err = rclcpp::spin_until_future_complete(service_client_node, result);
    if (err == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto cv_ptr = cv_bridge::toCvCopy(result.get()->image, "bgr8");
        cv::imshow("Madoka", cv_ptr->image);
        cv::waitKey(0);
    }
    else
    {
        std::cout << "Error " << rclcpp::to_string(err) << std::endl;
    }

    rclcpp::shutdown();
}