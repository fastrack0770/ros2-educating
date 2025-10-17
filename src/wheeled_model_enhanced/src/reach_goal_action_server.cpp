#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "wheeled_model_enhanced/pos.hpp"
#include "wheeled_model_enhanced/utils.hpp"

#include "wheeled_model_enhanced/action/reach_goal.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using ReachGoalAction = wheeled_model_enhanced::action::ReachGoal;
using GoalHandle = rclcpp_action::ServerGoalHandle<ReachGoalAction>;

// TODO make a speed publlisher that calculates speed from GPS position
// TODO replace all cout with the proper log

class ReachGoalActionServerNode : public rclcpp::Node
{
public:
    ReachGoalActionServerNode() : Node("reach_goal_action_server_node")
    {
        // navsat sub
        {
            const auto callback = [this](const sensor_msgs::msg::NavSatFix &msg)
            {
                _robot_pos = msg;
            };

            _navsat_sub = create_subscription<sensor_msgs::msg::NavSatFix>(
                "/navsat", 10, callback);
        }

        // imu sub
        {
            const auto callback = [this](const sensor_msgs::msg::Imu &msg)
            {
                _robot_imu = msg;

                try
                {
                    const auto w = _robot_imu.orientation.w;
                    const auto x = _robot_imu.orientation.x;
                    const auto y = _robot_imu.orientation.y;
                    const auto z = _robot_imu.orientation.z;

                    // euler z angle
                    const auto psi = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

                    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 100,
                                                "w: " << w << ", x: " << x << ", y: " << y << ", z: " << z << ", psi: " << psi);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Got exception while calculate angles from imu: " << e.what());
                }
            };

            _imu_sub = create_subscription<sensor_msgs::msg::Imu>(
                "/imu", 10, callback);
        }

        const auto handle_goal = [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ReachGoalAction::Goal> goal) -> rclcpp_action::GoalResponse
        {
            std::cout << "got goal "
                      << "lat: " << goal->goal_lat << " "
                      << "long: " << goal->goal_long << std::endl;
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
                            auto feedback = std::make_shared<ReachGoalAction::Feedback>();
                            auto result = std::make_shared<ReachGoalAction::Result>();
                            rclcpp::Rate loop_rate(1);

                            feedback->distance_to_point = utils::distance_in_meters(_robot_pos, Pos(_current_goal));

                            const auto start_point = rclcpp::Clock().now();

                            // TODO prevent an endless looping (?)
                            while (not is_goal_reached())
                            {
                                feedback->distance_to_point = utils::distance_in_meters(_robot_pos, Pos(_current_goal));

                                goal_handle->publish_feedback(feedback);
                                loop_rate.sleep();
                            }

                            result->elapsed_time = (rclcpp::Clock().now() - start_point).seconds();
                            goal_handle->succeed(result);

                            std::cout << "Goal Succeed" << std::endl; })
                .detach();
        };
        _action_server = rclcpp_action::create_server<ReachGoalAction>(this, "reach_goal",
                                                                       handle_goal,
                                                                       handle_cancel,
                                                                       handle_accepted);

        std::cout << "Reach goal action server is running" << std::endl;
    }

    bool is_goal_reached() const noexcept
    {
        // TODO get acceptable range from action server parameters
        constexpr double acceptable_range = 1; // 1 meter

        return utils::distance_in_meters(_robot_pos, Pos(_current_goal)) <= acceptable_range;
    }

private:
    rclcpp_action::Server<ReachGoalAction>::SharedPtr _action_server;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _navsat_sub;
    Pos _robot_pos;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    sensor_msgs::msg::Imu _robot_imu;

    std::shared_ptr<const ReachGoalAction::Goal> _current_goal;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReachGoalActionServerNode>());
    rclcpp::shutdown();

    return 0;
}