#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "udemy_ros2_pkg/action/navigate.hpp"

using NavigateAction = udemy_ros2_pkg::action::Navigate;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateAction>;
using Point = geometry_msgs::msg::Point;

class NavigateActionClientNode : public rclcpp::Node
{
public:
    NavigateActionClientNode() : Node("navigate_action_client_node")
    {
        _action_client = rclcpp_action::create_client<NavigateAction>(
            this, "navigate");

        prompt_user_for_goal();
    }
    ~NavigateActionClientNode()
    {
        _action_client->async_cancel_all_goals();
    }

    void prompt_user_for_goal()
    {
        auto goal_msg = NavigateAction::Goal();
        std::cout << "Enter x, y, z";
        std::cin >> goal_msg.goal_point.x >> goal_msg.goal_point.y >> goal_msg.goal_point.z;

        std::cout << "Wait for server" << std::endl;
        _action_client->wait_for_action_server();
        std::cout << "Sending goal" << std::endl;

        auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
        const auto goal_callback = [](GoalHandle::SharedPtr future)
        {
            auto goal_handle = future.get();

            if (!goal_handle)
            {
                std::cout << "Goal was rejected" << std::endl;
            }
            else
            {
                std::cout << "Goal accepted" << std::endl;
            }
        };
        send_goal_options.goal_response_callback = goal_callback;

        const auto feedback_callback = [](
                                           GoalHandle::SharedPtr,
                                           std::shared_ptr<const NavigateAction::Feedback> feedback)
        {
            std::cout << "Feedback: " << feedback->distance_to_point << std::endl;
        };
        send_goal_options.feedback_callback = feedback_callback;

        const auto result_callback = [](
                                         const GoalHandle::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                std::cout << "Time elapsed: " << result.result->elapsed_time << std::endl;
                break;
            case rclcpp_action::ResultCode::CANCELED:
            case rclcpp_action::ResultCode::ABORTED:
                std::cout << "Goal canceled/aborted";
                break;
            default:
                std::cout << "Unknown" << std::endl;
            }

            rclcpp::shutdown();
        };

        send_goal_options.result_callback = result_callback;

        _action_client->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavigateAction>::SharedPtr _action_client;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateActionClientNode>());
    rclcpp::shutdown();

    return 0;
}