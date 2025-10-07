#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "udemy_ros2_pkg/srv/turn.hpp"

using Turn = udemy_ros2_pkg::srv::Turn;

/**
 * TurnServiceNode
 * 
 * Service server created based on a service server example (service_server.cpp).
 * Waits for an angle from a "turn" topic.  
 * Then send a result (downloaded picture).  
 */
class TurnServiceNode : public rclcpp::Node
{
public:
    TurnServiceNode() : Node("turn_service_node")
    {
        const auto turn_payload = [](const Turn::Request::SharedPtr request,
                                       const Turn::Response::SharedPtr response)
        {
            std::cout << "Received angle " << request->angle << std::endl;
            auto image = cv::imread("/home/user/Pictures/KanameMadoka.png");
            
            auto image_msg_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
            response->image = *image_msg_ptr;
        };
        _service_server = create_service<Turn>(
            "turn",
            turn_payload);
    }

private:
    rclcpp::Service<Turn>::SharedPtr _service_server;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurnServiceNode>());
    rclcpp::shutdown();
}