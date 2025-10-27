#include <iomanip>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "wheeled_model_enhanced/action/reach_goal.hpp"
#include "wheeled_model_enhanced/storage.hpp"
#include "wheeled_model_enhanced/types.hpp"
#include "wheeled_model_enhanced/utils.hpp"

using ReachGoalAction = wheeled_model_enhanced::action::ReachGoal;
using GoalHandle = rclcpp_action::ServerGoalHandle<ReachGoalAction>;

class ReachGoalActionServerNode : public rclcpp::Node
{
  public:
    ReachGoalActionServerNode() : Node("reach_goal_action_server_node")
    {
        // robot navsat sub
        {
            const auto callback = [this](const sensor_msgs::msg::NavSatFix &msg) {
                _storage.set_robot_pos(msg);

                try
                {
                    RCLCPP_DEBUG_STREAM(get_logger(), std::setprecision(8) << "ROBOT gps: " << _storage.robot_gps_pos()
                                                                           << ", topoc: " << _storage.robot_topo_pos());
                    RCLCPP_DEBUG_STREAM(get_logger(),
                                        std::setprecision(8)
                                            << "distance gps: " << _storage.distance_to_waypoint_gps()
                                            << ", related: " << _storage.distance_to_waypoint_related()
                                            << ", angle: " << Degree(_storage.angle_to_waypoint()).value());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR_STREAM(get_logger(), "Got exception while calculate topocentric coords: " << e.what());
                }
            };

            _robot_navsat_sub =
                create_subscription<sensor_msgs::msg::NavSatFix>("/wheeled_model_enhanced/navsat", 10, callback);
        }

        // waypoint navsat sub
        {
            const auto callback = [this](const sensor_msgs::msg::NavSatFix &msg) {
                _storage.set_waypoint_pos(msg);

                try
                {
                    RCLCPP_DEBUG_STREAM(get_logger(), std::setprecision(8)
                                                          << "WAYPOINT gps: " << _storage.waypoint_gps_pos()
                                                          << ", topoc: " << _storage.waypoint_topo_pos()
                                                          << ", related: " << _storage.waypoint_related_pos());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR_STREAM(get_logger(), "Got exception while calculate topocentric coords: " << e.what());
                }
            };

            _waypoint_navsat_sub = create_subscription<sensor_msgs::msg::NavSatFix>("/waypoint/navsat", 10, callback);
        }

        // imu sub
        {
            const auto callback = [this](const sensor_msgs::msg::Imu &msg) {
                _storage.set_imu(msg);

                try
                {
                    RCLCPP_DEBUG_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
                                                 "robot azimuth: " << _storage.robot_azimuth());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
                                                 "Got exception while calculate angles from imu: " << e.what());
                }
            };

            _imu_sub = create_subscription<sensor_msgs::msg::Imu>("/imu", 10, callback);
        }

        // speed pub
        {
            _speed_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }

        const auto handle_goal =
            [this](const rclcpp_action::GoalUUID &,
                   std::shared_ptr<const ReachGoalAction::Goal> goal) -> rclcpp_action::GoalResponse {
            if (_is_running)
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Reject goal due the goal already is");

                return rclcpp_action::GoalResponse::REJECT;
            }

            _is_running = true; // prepare to run threads

            RCLCPP_DEBUG_STREAM(get_logger(), "Got goal: " << "lat: " << goal->goal_lat << " "
                                                           << "long: " << goal->goal_long
                                                           << ".Skipping goal, going to the waypoint instead");

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };
        const auto handle_cancel = [this](std::shared_ptr<GoalHandle>) -> rclcpp_action::CancelResponse {
            RCLCPP_WARN_STREAM(get_logger(), "The goal has been cancelled");

            _is_running = false;

            {
                geometry_msgs::msg::Twist msg_to_pub;
                msg_to_pub.angular.z = 0;
                _speed_pub->publish(msg_to_pub);
                RCLCPP_INFO_STREAM(get_logger(), "The robot has been stopped");
            }

            return rclcpp_action::CancelResponse::ACCEPT;
        };
        const auto handle_accepted = [this](std::shared_ptr<GoalHandle> goal_handle) {
            auto thread = std::thread([this, goal_handle]() {
                RCLCPP_INFO_STREAM(get_logger(), "Execute goal");

                const auto goal = goal_handle->get_goal();
                auto feedback = std::make_shared<ReachGoalAction::Feedback>();
                auto result = std::make_shared<ReachGoalAction::Result>();
                rclcpp::Rate loop_rate(1);

                feedback->distance_to_point = _storage.distance_to_waypoint_gps().value();

                const auto start_point = rclcpp::Clock().now();

                // calculate distances to turn to
                const double a_max = 2;                      // rad/sec^2 ???
                const double v_max = 1;                      // rad/sec   ???
                const auto s_ac = pow(v_max, 2) / 2 * a_max; // distance, after which the velocity will become maximum
                const auto angle_to_waypoint = _storage.angle_to_waypoint();
                std::thread control_thread;

                if (not _is_running)
                {
                    RCLCPP_INFO_STREAM(get_logger(), "Stop executing due goal is cancelled");
                    return;
                }

                if (s_ac * 2 >= angle_to_waypoint.value())
                {
                    const auto half_distance = abs(angle_to_waypoint.value() / 2);
                    const auto velocity_to_set =
                        sqrt(half_distance * 2 * a_max) * utils::sign(angle_to_waypoint.value());
                    const auto acceleration_time =
                        abs(velocity_to_set / a_max); // after that time we need to set velocity to zero

                    RCLCPP_INFO_STREAM(get_logger(), "Turn to " << angle_to_waypoint << ", velocity " << velocity_to_set
                                                                << ", acceleration time " << acceleration_time
                                                                << ", north angle " << _storage.robot_azimuth());

                    {
                        geometry_msgs::msg::Twist msg_to_pub;
                        msg_to_pub.angular.z = velocity_to_set * (-1);
                        _speed_pub->publish(msg_to_pub);
                        RCLCPP_INFO_STREAM(get_logger(), "Set velocity to " << velocity_to_set);
                    }

                    control_thread = std::thread([this, acceleration_time, start_point]() {
                        rclcpp::Rate loop_rate(60); // TODO find exact number
                        while (_is_running and
                               rclcpp::Clock().now() - start_point < std::chrono::duration<double>(acceleration_time))
                        {
                            loop_rate.sleep();
                        }

                        if (not _is_running)
                        {
                            RCLCPP_INFO_STREAM(get_logger(), "Stop executing inner thread due goal is cancelled");
                            return;
                        }

                        {
                            geometry_msgs::msg::Twist msg_to_pub;
                            msg_to_pub.angular.z = 0;
                            _speed_pub->publish(msg_to_pub);
                            RCLCPP_INFO_STREAM(get_logger(), "Set velocity to 0");
                        }
                    });
                }
                else
                {
                    const auto acceleration_time = v_max / a_max;
                    const auto time_with_max_speed = (abs(angle_to_waypoint.value()) - 2 * s_ac) / v_max;
                    const auto time_before_stopping = acceleration_time + time_with_max_speed;
                    const auto velocity_to_set = v_max * utils::sign(angle_to_waypoint.value());

                    RCLCPP_INFO_STREAM(get_logger(), "Turn to " << angle_to_waypoint << ", velocity " << velocity_to_set
                                                                << ", time_before_stopping " << time_before_stopping
                                                                << ", north angle " << _storage.robot_azimuth());

                    {
                        geometry_msgs::msg::Twist msg_to_pub;
                        msg_to_pub.angular.z = velocity_to_set * (-1);
                        _speed_pub->publish(msg_to_pub);
                    }

                    // TODO duplication - can make it better
                    control_thread = std::thread([this, time_before_stopping, start_point]() {
                        rclcpp::Rate loop_rate(60); // TODO find exact number
                        while (_is_running and rclcpp::Clock().now() - start_point <
                                                   std::chrono::duration<double>(time_before_stopping))
                        {
                            loop_rate.sleep();
                        }

                        if (not _is_running)
                        {
                            RCLCPP_INFO_STREAM(get_logger(), "Stop executing inner thread due goal is cancelled");
                            return;
                        }

                        {
                            geometry_msgs::msg::Twist msg_to_pub;
                            msg_to_pub.angular.z = 0;
                            _speed_pub->publish(msg_to_pub);
                        }
                    });
                }

                while (_is_running and not is_goal_reached())
                {
                    feedback->distance_to_point = _storage.distance_to_waypoint_gps().value();

                    goal_handle->publish_feedback(feedback);
                    loop_rate.sleep();
                }

                if (not _is_running)
                {
                    RCLCPP_INFO_STREAM(get_logger(), "Stop executing main loop due goal is cancelled");
                    if (control_thread.joinable())
                    {
                        control_thread.join();
                    }
                    return;
                }

                result->elapsed_time = (rclcpp::Clock().now() - start_point).seconds();
                goal_handle->succeed(result);

                RCLCPP_INFO_STREAM(get_logger(), "Goal Succeed");

                if (control_thread.joinable())
                {
                    control_thread.join();
                }
            });

            thread.detach();
        };
        _action_server = rclcpp_action::create_server<ReachGoalAction>(this, "reach_goal", handle_goal, handle_cancel,
                                                                       handle_accepted);

        RCLCPP_INFO_STREAM(get_logger(), "Reach goal action server is running");
    }

    bool is_goal_reached() const noexcept
    {
        // TODO get acceptable range from action server parameters
        constexpr Meter acceptable_range = 1; // 1 meter

        return _storage.distance_to_waypoint_gps() <= acceptable_range;
    }

  private:
    rclcpp_action::Server<ReachGoalAction>::SharedPtr _action_server;

    Storage _storage;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _robot_navsat_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _waypoint_navsat_sub;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _speed_pub;

    std::atomic_bool _is_running{false};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReachGoalActionServerNode>());
    rclcpp::shutdown();

    return 0;
}