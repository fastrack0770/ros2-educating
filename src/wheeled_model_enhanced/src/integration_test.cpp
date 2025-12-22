#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "wheeled_model_enhanced/action/reach_goal.hpp"

#include <queue>

using ReachGoalAction = wheeled_model_enhanced::action::ReachGoal;
using GoalHandle = rclcpp_action::ClientGoalHandle<ReachGoalAction>;

std::ostream &operator<<(std::ostream &os, const ReachGoalAction::Goal &rhv)
{
    return os << "(" << rhv.goal_lat << "; " << rhv.goal_long << ")";
}

/**
 * IntegrationTestNode
 *
 * To make a robot to move in different directions and angles to check
 * if all movemennts is alright
 */
class IntegrationTestNode : public rclcpp::Node
{
    enum class ResultCode
    {
        NO_CODE,
        SUCCEEDED,
        CANCELED,
        ABORTED,
        REJECTED,
    };

    ResultCode to_result_code(rclcpp_action::ResultCode code)
    {
        switch (code)
        {
        case rclcpp_action::ResultCode::ABORTED:
            return ResultCode::ABORTED;
        case rclcpp_action::ResultCode::SUCCEEDED:
            return ResultCode::SUCCEEDED;
        case rclcpp_action::ResultCode::CANCELED:
            return ResultCode::CANCELED;
        default:
            return ResultCode::NO_CODE;
        }
    }

    std::string to_text(ResultCode code)
    {
        switch (code)
        {
        case ResultCode::SUCCEEDED:
            return "SUCCEEDED";
        case ResultCode::CANCELED:
            return "CANCELED";
        case ResultCode::ABORTED:
            return "ABORTED";
        case ResultCode::REJECTED:
            return "REJECTED";
        default:
            return "NO CODE";
        }
    }

    struct GoalResult
    {
        ReachGoalAction::Goal goal;
        ResultCode result_code = ResultCode::NO_CODE;
        float elapsed_time = -1.f;
    };

  public:
    IntegrationTestNode() : Node("integration_test_node")
    {
        _action_client = rclcpp_action::create_client<ReachGoalAction>(this, "reach_goal");

        fill_queue();

        prompt_user_for_goal();
    }

    ~IntegrationTestNode()
    {
        _action_client->async_cancel_all_goals();
    }

    void fill_queue()
    {
        const auto create_goal = [](double latitude, double longitude) {
            auto goal = ReachGoalAction::Goal();
            goal.goal_lat = latitude;
            goal.goal_long = longitude;

            return goal;
        };

        _goal_queue.push(create_goal(-0.40119337, -0.75402623)); // -2; 0 
        _goal_queue.push(create_goal(-0.4011918, -0.75402419));  // 10; 10
        _goal_queue.push(create_goal(-0.40119337, -0.75402419)); // 10; 0 
        _goal_queue.push(create_goal(-0.40119495, -0.75402419)); // 10; -10 
        _goal_queue.push(create_goal(-0.40119495, -0.75402589)); // 0; -10
        _goal_queue.push(create_goal(-0.40119495, -0.75402759)); // -10; -10
        _goal_queue.push(create_goal(-0.40119337, -0.75402759)); // -10; 0
        _goal_queue.push(create_goal(-0.4011918, -0.75402759)); // -10; 10 
        _goal_queue.push(create_goal(-0.4011918, -0.75402589)); // 0; 10 
    }

    void prompt_user_for_goal()
    {
        if (not _goal_queue.empty())
        {
            auto goal = _goal_queue.front();
            go_to_goal(goal);

            _goal_queue.pop();
        }
        else
        {
            evaluate_results();
            rclcpp::shutdown();
        }
    }

    void evaluate_results()
    {
        while (not _result_queue.empty())
        {
            const auto result = _result_queue.front();

            if (result.result_code == ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Goal " << result.goal << " - " << to_text(result.result_code)
                                                         << ", time elapsed " << result.elapsed_time << " sec");
            }
            else
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Goal " << result.goal << " - " << to_text(result.result_code));
            }

            _result_queue.pop();
        }
    }

    void go_to_goal(ReachGoalAction::Goal goal_msg)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Wait for server");
        _action_client->wait_for_action_server();

        RCLCPP_INFO_STREAM(get_logger(), "Sending goal with parameters in rads lat: " << goal_msg.goal_lat << ", long: "
                                                                                      << goal_msg.goal_long);
        auto send_goal_options = rclcpp_action::Client<ReachGoalAction>::SendGoalOptions();
        const auto goal_callback = [this, goal_msg](GoalHandle::SharedPtr future) {
            auto goal_handle = future.get();

            if (!goal_handle)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Goal was rejected");

                _result_queue.push({goal_msg, ResultCode::REJECTED, -1});
            }
            else
            {
                RCLCPP_INFO_STREAM(get_logger(), "Goal was accepted");
            }
        };
        send_goal_options.goal_response_callback = goal_callback;

        const auto feedback_callback = [this](GoalHandle::SharedPtr,
                                              std::shared_ptr<const ReachGoalAction::Feedback> feedback) {
            RCLCPP_INFO_STREAM(get_logger(), "Meters to the goal: " << feedback->distance_to_point);
        };
        send_goal_options.feedback_callback = feedback_callback;

        const auto result_callback = [this, goal_msg](const GoalHandle::WrappedResult &result) {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO_STREAM(get_logger(), "Time elapsed: " << result.result->elapsed_time);
                break;
            case rclcpp_action::ResultCode::CANCELED:
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO_STREAM(get_logger(), "Goal was canceled/aborted");
                break;
            default:
                RCLCPP_INFO_STREAM(get_logger(), "Unknown");
            }

            _result_queue.push({goal_msg, to_result_code(result.code), result.result->elapsed_time});

            prompt_user_for_goal();
        };

        send_goal_options.result_callback = result_callback;

        _action_client->async_send_goal(goal_msg, send_goal_options);
    }

  private:
    rclcpp_action::Client<ReachGoalAction>::SharedPtr _action_client;

    std::queue<ReachGoalAction::Goal> _goal_queue;
    std::queue<GoalResult> _result_queue;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntegrationTestNode>());
    rclcpp::shutdown();

    return 0;
}