#include <iomanip>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "wheeled_model_enhanced/action/reach_goal.hpp"
#include "wheeled_model_enhanced/storage.hpp"
#include "wheeled_model_enhanced/types.hpp"
#include "wheeled_model_enhanced/utils.hpp"

using ReachGoalAction = wheeled_model_enhanced::action::ReachGoal;
using GoalHandle = rclcpp_action::ServerGoalHandle<ReachGoalAction>;

namespace params
{
constexpr const char *angle_threshold = "angle_threshold";
constexpr const double angle_threshold_default = 0.05;

constexpr const char *distance_threshold = "distance_threshold";
constexpr const double distance_threshold_default = 0.5;

constexpr const char *max_angle_acceleration = "max_angle_acceleration";
constexpr const double max_angle_acceleration_default = 2;

constexpr const char *max_angle_velocity = "max_angle_velocity";
constexpr const double max_angle_velocity_default = 1;

// Difference between the IMU sensor's x axe and the robot forward direction
constexpr const char *robot_imu_twist = "robot_imu_twist";
constexpr const double robot_imu_twist_default = 1.5707963267948966; // 90 in degrees

constexpr const char *max_acceleration = "max_acceleration";
constexpr const double max_acceleration_default = 1;

constexpr const char *max_velocity = "max_velocity";
constexpr const double max_velocity_default = 10;

constexpr const char *robot_length_in_m = "robot_length_in_m";
constexpr const double robot_length_in_m_default = 1.5;
} // end of namespace params

class ReachGoalActionServerNode : public rclcpp_lifecycle::LifecycleNode
{
  public:
    explicit ReachGoalActionServerNode(bool intra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode("reach_goal_action_server_node",
                                          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_configure() is called.");

        // parameters
        declare_parameter<double>(params::angle_threshold, params::angle_threshold_default);
        declare_parameter<double>(params::distance_threshold, params::distance_threshold_default);
        declare_parameter<double>(params::max_angle_acceleration, params::max_angle_acceleration_default);
        declare_parameter<double>(params::max_angle_velocity, params::max_angle_velocity_default);
        declare_parameter<double>(params::robot_imu_twist, params::robot_imu_twist_default);
        declare_parameter<double>(params::max_acceleration, params::max_acceleration_default);
        declare_parameter<double>(params::max_velocity, params::max_velocity_default);
        declare_parameter<double>(params::robot_length_in_m, params::robot_length_in_m_default);

        _storage.set_robot_imu_twist(get_parameter(params::robot_imu_twist).as_double());
        _storage.set_robot_length(get_parameter(params::robot_length_in_m).as_double());

        // robot navsat sub
        {
            const auto callback = [this](const sensor_msgs::msg::NavSatFix &msg) {
                _storage.set_robot_pos(msg);

                try
                {
                    RCLCPP_DEBUG_STREAM(get_logger(), std::setprecision(8)
                                                          << "ROBOT gps: " << _storage.robot_gps_pos()
                                                          << ", topoc: " << _storage.robot_topo_pos()
                                                          << ", related: " << _storage.robot_related_pos());
                    RCLCPP_DEBUG_STREAM(get_logger(), std::setprecision(8)
                                                          << "distance gps: " << _storage.distance_to_waypoint_gps()
                                                          << ", related: " << _storage.distance_to_waypoint_related()
                                                          << ", angle to wp (rad): " << _storage.angle_to_waypoint());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR_STREAM(get_logger(), "Got exception while calculate topocentric coords: " << e.what());
                }
            };

            _robot_navsat_sub =
                create_subscription<sensor_msgs::msg::NavSatFix>("/wheeled_model_enhanced/navsat", 10, callback);
        }

        // imu sub
        {
            const auto callback = [this](const sensor_msgs::msg::Imu &msg) {
                _storage.set_imu(msg);

                try
                {
                    RCLCPP_DEBUG_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
                                                 "robot azimuth: " << _storage.robot_azimuth() << ", angle to waypoint "
                                                                   << _storage.angle_to_waypoint());
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
            rclcpp::QoS qos(10);
            qos.reliable();

            rclcpp::PublisherOptionsBase options_base;
            options_base.event_callbacks.deadline_callback = [this](const rclcpp::QOSDeadlineOfferedInfo &info) {
                RCLCPP_ERROR_STREAM(get_logger(), "Speed publisher - deadline missed, total count "
                                                      << info.total_count << ", total count change "
                                                      << info.total_count_change);
            };
            options_base.event_callbacks.incompatible_qos_callback =
                [this](const rclcpp::QOSOfferedIncompatibleQoSInfo &info) {
                    RCLCPP_ERROR_STREAM(get_logger(), "Speed publisher - offered incompatible qos, total count "
                                                          << info.total_count << ", total count change "
                                                          << info.total_count_change << ", last_policy_kind "
                                                          << info.last_policy_kind);
                };
            options_base.event_callbacks.liveliness_callback = [this](const rclcpp::QOSLivelinessLostInfo &info) {
                RCLCPP_ERROR_STREAM(get_logger(), "Speed publisher - liveliness lost, total count "
                                                      << info.total_count << ", total count change "
                                                      << info.total_count_change);
            };

            rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options(options_base);

            _speed_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos, options);

            _speed_pub_timer = create_wall_timer(std::chrono::milliseconds(1000), [this]() {
                geometry_msgs::msg::Twist msg_to_pub;
                msg_to_pub.linear.x = _linear_speed;
                msg_to_pub.angular.z =
                    _angular_speed * (-1); // we assume that a positive speed is for turning clockwise

                _speed_pub->publish(msg_to_pub);

                RCLCPP_INFO_STREAM(get_logger(), "Set velocity to " << msg_to_pub);
            });
        }

        // odometry sub
        {
            const auto callback = [this](const nav_msgs::msg::Odometry &msg) {
                _storage.set_odometry(msg);

                try
                {
                    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
                                                "angular speed: " << _storage.angular_speed()
                                                                  << ", angular ac: " << _storage.angular_acceleration()
                                                                  << ", linear speed: " << _storage.linear_speed()
                                                                  << ", linear ac: " << _storage.linear_acceleration());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
                                                 "Got exception while calculate angles from odometry: " << e.what());
                }
            };

            _odometry_sub =
                create_subscription<nav_msgs::msg::Odometry>("/model/wheeled_model_enhanced/odometry", 10, callback);
        }

        // lidar sub
        {
            const auto callback = [this](const sensor_msgs::msg::LaserScan &) {
                try
                {
                    RCLCPP_DEBUG_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "got lidar info");
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
                                                 "Got exception while getting lidar info: " << e.what());
                }
            };

            _lidar_sub = create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, callback);
        }

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &state)
    {
        LifecycleNode::on_activate(state);
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

        const auto handle_goal =
            [this](const rclcpp_action::GoalUUID &,
                   std::shared_ptr<const ReachGoalAction::Goal> goal) -> rclcpp_action::GoalResponse {
            if (not is_active())
            {
                RCLCPP_ERROR_STREAM(get_logger(),
                                    "Reject goal due the server is not in activate state, current state is - "
                                        << LifecycleNode::get_current_state().label());

                return rclcpp_action::GoalResponse::REJECT;
            }
            if (_is_running)
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Reject goal due the goal already is");

                return rclcpp_action::GoalResponse::REJECT;
            }

            _storage.set_waypoint_pos(goal);

            _is_running = true; // prepare to run threads

            RCLCPP_DEBUG_STREAM(get_logger(), "Got goal: " << "lat: " << goal->goal_lat << " "
                                                           << "long: " << goal->goal_long
                                                           << ".Skipping goal, going to the waypoint instead");

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };
        const auto handle_cancel = [this](std::shared_ptr<GoalHandle>) -> rclcpp_action::CancelResponse {
            RCLCPP_WARN_STREAM(get_logger(), "The goal has been cancelled");

            _is_running = false;

            set_robot_angle_speed(0);
            set_robot_speed(0);

            return rclcpp_action::CancelResponse::ACCEPT;
        };
        const auto handle_accepted = [this](std::shared_ptr<GoalHandle> goal_handle) {
            auto thread = std::thread([this, goal_handle]() {
                RCLCPP_INFO_STREAM(get_logger(), "Execute goal");

                const auto goal = goal_handle->get_goal();
                auto feedback = std::make_shared<ReachGoalAction::Feedback>();
                auto result = std::make_shared<ReachGoalAction::Result>();

                rclcpp::Rate loop_rate(1);

                feedback->distance_to_point = _storage.distance_to_waypoint_gps().to_double();

                const auto start_point = rclcpp::Clock().now();

                std::thread control_thread;
                // turn towards the waypoint
                {
                    const auto a_max = Radian(get_parameter(params::max_angle_acceleration).as_double()); // rad/sec^2
                    const auto v_max = Radian(get_parameter(params::max_angle_velocity).as_double());     // rad/sec

                    if (not _is_running)
                    {
                        RCLCPP_INFO_STREAM(get_logger(), "Stop executing due goal is cancelled");
                        return;
                    }

                    if (not is_angle_reached())
                    {
                        control_thread = std::thread([this, v_max, a_max]() {
                            rclcpp::Rate loop_rate(60);

                            while (_is_running and not is_angle_reached())
                            {
                                const auto angle_to_waypoint = _storage.angle_to_waypoint();

                                const auto [velocity_to_set, s_ac] = utils::get_speed(v_max, a_max, angle_to_waypoint);

                                RCLCPP_INFO_STREAM(get_logger(), "Turn to " << angle_to_waypoint << ", velocity "
                                                                            << velocity_to_set << ", north angle "
                                                                            << _storage.robot_azimuth());

                                set_robot_angle_speed(velocity_to_set);

                                while (_is_running and fabs(_storage.angle_to_waypoint().to_double()) > s_ac)
                                {
                                    loop_rate.sleep();
                                }

                                stop_robot();

                                while (_is_running and _storage.has_angular_speed())
                                {
                                    loop_rate.sleep();
                                }
                            }
                        });
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(get_logger(), "Angle is already reached");
                    }

                    while (_is_running and not is_angle_reached())
                    {
                        feedback->distance_to_point = _storage.distance_to_waypoint_gps().to_double();

                        goal_handle->publish_feedback(feedback);
                        loop_rate.sleep();
                    }

                    if (control_thread.joinable())
                    {
                        control_thread.join();
                    }

                    if (not _is_running)
                    {
                        RCLCPP_INFO_STREAM(get_logger(), "Not running, stop goal execution");
                        return;
                    }
                }

                // go to the waypoint
                {
                    const auto a_max = get_parameter(params::max_acceleration).as_double(); // meter/sec^2
                    const auto v_max = get_parameter(params::max_velocity).as_double();     // meter/sec

                    if (not _is_running)
                    {
                        RCLCPP_INFO_STREAM(get_logger(), "Stop executing due goal is cancelled");
                        return;
                    }

                    if (not is_goal_reached())
                    {
                        control_thread = std::thread([this, start_point, v_max, a_max]() {
                            rclcpp::Rate loop_rate(60);

                            while (_is_running and not is_goal_reached())
                            {
                                const auto distance_to_waypoint = _storage.distance_to_waypoint_related().to_double();

                                const auto [velocity_to_set, s_ac] =
                                    utils::get_speed(v_max, a_max, distance_to_waypoint);

                                RCLCPP_INFO_STREAM(get_logger(), "Go to waypoint, velocity " << velocity_to_set
                                                                                             << ", distance "
                                                                                             << distance_to_waypoint);

                                set_robot_speed(velocity_to_set);

                                while (_is_running and _storage.distance_to_waypoint_related() > s_ac)
                                {
                                    loop_rate.sleep();
                                }

                                set_robot_speed(0);

                                while (_is_running and _storage.has_linear_speed())
                                {
                                    loop_rate.sleep();
                                }
                            }
                        });
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(get_logger(), "Destination is already reached");
                    }

                    while (_is_running and not is_goal_reached())
                    {
                        feedback->distance_to_point = _storage.distance_to_waypoint_related().to_double();

                        goal_handle->publish_feedback(feedback);
                        loop_rate.sleep();
                    }

                    if (control_thread.joinable())
                    {
                        control_thread.join();
                    }

                    if (not _is_running)
                    {
                        RCLCPP_INFO_STREAM(get_logger(), "Not running, stop goal execution");
                        return;
                    }
                }

                result->elapsed_time = (rclcpp::Clock().now() - start_point).seconds();
                goal_handle->succeed(result);

                RCLCPP_INFO_STREAM(get_logger(), "Goal Succeed");

                _is_running = false;
            });

            thread.detach();
        };

        // waiting for server to be initialized with initial data
        if (_storage.initialized())
        {
            _action_server = rclcpp_action::create_server<ReachGoalAction>(this, "reach_goal", handle_goal,
                                                                           handle_cancel, handle_accepted);

            RCLCPP_INFO_STREAM(get_logger(), "Reach goal action server is running");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        RCLCPP_ERROR_STREAM(get_logger(), "Reach goal action server didn't initialized, aborting activation");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &state)
    {
        LifecycleNode::on_deactivate(state);

        RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

        _is_running = false;

        _action_server.reset();

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &)
    {
        RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &state)
    {
        RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    bool is_active()
    {
        return LifecycleNode::get_current_state().label() == "active";
    }

  private:
    /**
     * set_robot_angle_speed
     * args - speed in rad/sec
     */
    void set_robot_angle_speed(double speed)
    {
        _angular_speed = speed;

        _speed_pub_timer->execute_callback();
    }

    /**
     * set_robot_speed
     * args - speed in meter/sec
     */
    void set_robot_speed(double speed)
    {
        _linear_speed = speed;

        _speed_pub_timer->execute_callback();
    }

    void stop_robot()
    {
        _linear_speed = 0;
        _angular_speed = 0;

        _speed_pub_timer->execute_callback();
    }

    /**
     * Used only by
     */
    void set_robot_speed()
    {
    }

    bool is_angle_reached() const noexcept
    {
        const auto acceptable_range = Radian(get_parameter(params::angle_threshold).as_double());

        return fabs(_storage.angle_to_waypoint().to_double()) <= acceptable_range.to_double();
    }

    bool is_goal_reached() const noexcept
    {
        const auto acceptable_range = Meter(get_parameter(params::distance_threshold).as_double());

        return _storage.distance_to_waypoint_related() <= acceptable_range;
    }

  private:
    rclcpp_action::Server<ReachGoalAction>::SharedPtr _action_server;

    Storage _storage;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _robot_navsat_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _speed_pub;
    rclcpp::TimerBase::SharedPtr _speed_pub_timer;
    std::atomic<double> _linear_speed{0.f};
    std::atomic<double> _angular_speed{0.f};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_sub;

    std::atomic_bool _is_running{false};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    auto lc_node = std::make_shared<ReachGoalActionServerNode>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}