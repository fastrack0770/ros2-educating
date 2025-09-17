#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "udemy_ros2_pkg/action/navigate.hpp"

using NavigateAction = udemy_ros2_pkg::action::Navigate;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateAction>;
using Point = geometry_msgs::msg::Point;

// Close enough to the goal
const auto DIST_THRESHOLD = 0.5;

class NavigateActionServerNode : public rclcpp::Node
{
public:
    NavigateActionServerNode() : Node("navigate_action_server_node")
    {
        _robot_pos = Point();
        _robot_pos_sub = create_subscription<Point>(
            "robot_position", 10,
            [this](const Point &point)
            {
                _robot_pos = point;
            });
        const auto handle_goal = [](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateAction::Goal> goal) -> rclcpp_action::GoalResponse
        {
            std::cout << "got goal "
                      << goal->goal_point.x << " "
                      << goal->goal_point.y << " "
                      << goal->goal_point.z << std::endl;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };
        const auto handle_cancel = [](std::shared_ptr<GoalHandle>) -> rclcpp_action::CancelResponse
        {
            std::cout << "goal has been cancelled" << std::endl;

            return rclcpp_action::CancelResponse::ACCEPT;
        };
        const auto handle_accepted = [this](std::shared_ptr<GoalHandle> goal_handle)
        {
            std::thread([this, goal_handle]()
                        {
                            std::cout << "Execute goal" << std::endl;

                            const auto goal = goal_handle->get_goal();
                            auto feedback = std::make_shared<NavigateAction::Feedback>();
                            auto result = std::make_shared<NavigateAction::Result>();
                            rclcpp::Rate loop_rate(1);


                            feedback->distance_to_point = DIST_THRESHOLD; // Delete later? Loop must shoot once
                            const auto start_point = rclcpp::Clock().now();

                            while (feedback->distance_to_point >= DIST_THRESHOLD)
                            {
                                feedback->distance_to_point = std::sqrt(
                                    std::pow(_robot_pos.x - goal->goal_point.x, 2) +
                                    std::pow(_robot_pos.y - goal->goal_point.y, 2) +
                                    std::pow(_robot_pos.z - goal->goal_point.z, 2));

                                goal_handle->publish_feedback(feedback);
                                loop_rate.sleep();
                            }

                            result->elapsed_time = (rclcpp::Clock().now() - start_point).seconds();
                            goal_handle->succeed(result);

                            std::cout << "Goal Succeed" << std::endl; })
                .detach();
        };
        _action_server = rclcpp_action::create_server<NavigateAction>(this, "navigate",
                                                                      handle_goal,
                                                                      handle_cancel,
                                                                      handle_accepted);

        std::cout << "Action server is running" << std::endl;
    }

private:
    rclcpp_action::Server<NavigateAction>::SharedPtr _action_server;
    Point _robot_pos;
    rclcpp::Subscription<Point>::SharedPtr _robot_pos_sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateActionServerNode>());
    rclcpp::shutdown();

    return 0;
}