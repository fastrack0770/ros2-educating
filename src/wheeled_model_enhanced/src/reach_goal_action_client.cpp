#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "wheeled_model_enhanced/action/reach_goal.hpp"

using ReachGoalAction = wheeled_model_enhanced::action::ReachGoal;
using GoalHandle = rclcpp_action::ClientGoalHandle<ReachGoalAction>;

/**
 * ReachGoalActionClientNode
 */
class ReachGoalActionClientNode : public rclcpp::Node
{
  public:
    ReachGoalActionClientNode() : Node("reach_goal_action_client_node")
    {
        _action_client = rclcpp_action::create_client<ReachGoalAction>(this, "reach_goal");

        prompt_user_for_goal();
    }
    ~ReachGoalActionClientNode()
    {
        _action_client->async_cancel_all_goals();
    }

    void prompt_user_for_goal()
    {
        auto goal_msg = ReachGoalAction::Goal();

        RCLCPP_INFO_STREAM(get_logger(), "Wait for server");
        _action_client->wait_for_action_server();

        RCLCPP_INFO_STREAM(get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<ReachGoalAction>::SendGoalOptions();
        const auto goal_callback = [this](GoalHandle::SharedPtr future) {
            auto goal_handle = future.get();

            if (!goal_handle)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Goal was rejected");
            }
            else
            {
                RCLCPP_INFO_STREAM(get_logger(), "Goal accepted");
            }
        };
        send_goal_options.goal_response_callback = goal_callback;

        const auto feedback_callback = [this](GoalHandle::SharedPtr,
                                          std::shared_ptr<const ReachGoalAction::Feedback> feedback) {
            RCLCPP_INFO_STREAM(get_logger(), "Feedback: " << feedback->distance_to_point);
        };
        send_goal_options.feedback_callback = feedback_callback;

        const auto result_callback = [this](const GoalHandle::WrappedResult &result) {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO_STREAM(get_logger(), "Time elapsed: " << result.result->elapsed_time);
                break;
            case rclcpp_action::ResultCode::CANCELED:
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO_STREAM(get_logger(), "Goal canceled/aborted");
                break;
            default:
                RCLCPP_INFO_STREAM(get_logger(), "Unknown");
            }

            rclcpp::shutdown();
        };

        send_goal_options.result_callback = result_callback;

        _action_client->async_send_goal(goal_msg, send_goal_options);
    }

  private:
    rclcpp_action::Client<ReachGoalAction>::SharedPtr _action_client;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReachGoalActionClientNode>());
    rclcpp::shutdown();

    return 0;
}