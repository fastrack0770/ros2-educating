#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "wheeled_model_enhanced/action/reach_goal.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <cmath>
#include <atomic>
#include <mutex>

using ReachGoalAction = wheeled_model_enhanced::action::ReachGoal;
using GoalHandle = rclcpp_action::ServerGoalHandle<ReachGoalAction>;

/**
 * Pos
 * Thread-safe GPS position holder
 */
struct Pos
{
    Pos() {}

    Pos(std::shared_ptr<const ReachGoalAction::Goal> goal)
        : _latitude(goal->goal_lat), _longitude(goal->goal_long), _altitude(0.f)
    {
    }
    double latitude() const
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        return _latitude;
    }

    double longitude() const
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        return _longitude;
    }

    double altitude() const
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        return _altitude;
    }

    void setLatitude(double new_lat)
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        _latitude = new_lat;
    }

    void setLongitude(double new_long)
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        _longitude = new_long;
    }

    void setAltitude(double new_alt)
    {
        const std::lock_guard<decltype(_m)> lock(_m);
        _altitude = new_alt;
    }

    Pos &operator=(const sensor_msgs::msg::NavSatFix &msg)
    {
        const std::lock_guard<decltype(_m)> lock(_m);

        _latitude = msg.latitude;
        _longitude = msg.longitude;
        _altitude = msg.altitude;
        return *this;
    }

private:
    mutable std::mutex _m;

    double _latitude = 0.f;
    double _longitude = 0.f;
    double _altitude = 0.f;
};

/**
 * get_vector_3d_abs
 * Get a module from a 3d vector
 */
double get_vector_3d_abs(double x, double y, double z)
{
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
}

/**
 * earth_radius_at
 * Earth radius at the specified latitude
 * Realization got from: https://gis.stackexchange.com/questions/242188/calculating-the-earth-radius-at-latitude
 */
double earth_radius_at(const Pos &pos)
{
    constexpr double WGS_ELLIPSOID_EQ = 6378137.0;    // equatorial, in meters
    constexpr double WGS_ELLIPSOID_POL = 6356752.314; // polar, in meters

    const auto f1 = std::pow(std::pow(WGS_ELLIPSOID_EQ, 2) * std::cos(pos.latitude()), 2);
    const auto f2 = std::pow(std::pow(WGS_ELLIPSOID_POL, 2) * std::sin(pos.latitude()), 2);
    const auto f3 = std::pow(WGS_ELLIPSOID_EQ * std::cos(pos.latitude()), 2);
    const auto f4 = std::pow(WGS_ELLIPSOID_POL * std::sin(pos.latitude()), 2);

    const auto radius = std::sqrt((f1 + f2) / (f3 + f4));

    return radius;
}

// TODO make tests for it to be sure the ouput is in meters
/**
 * distance_in_meters
 * Distance by Haversine formula
 * Source: https://www.movable-type.co.uk/scripts/latlong.html
 */
double distance_in_meters(const Pos &lhv, const Pos &rhv)
{
    const double a = std::pow(std::sin(std::abs(lhv.latitude() - rhv.latitude()) / 2), 2) +
                     std::cos(lhv.latitude()) * std::cos(rhv.latitude()) *
                         std::pow(std::sin(std::abs(lhv.longitude() - rhv.longitude()) / 2), 2);
    const double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    const double earth_radius = earth_radius_at(lhv); // in meters
    const double d = earth_radius * c;

    return d;
}

// TODO make a speed publlisher that calculates speed from GPS position
// TODO replace all cout with the proper log

class ReachGoalActionServerNode : public rclcpp::Node
{
public:
    ReachGoalActionServerNode() : Node("reach_goal_action_server_node")
    {
        const auto callback = [this](const sensor_msgs::msg::NavSatFix &msg)
        {
            _robot_pos = msg;
        };

        _navsat_sub = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/navsat", 10, callback);

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

                            feedback->distance_to_point = distance_in_meters(_robot_pos, Pos(_current_goal));

                            const auto start_point = rclcpp::Clock().now();

                            while (not is_goal_reached())
                            {
                                feedback->distance_to_point = distance_in_meters(_robot_pos, Pos(_current_goal));

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

        return distance_in_meters(_robot_pos, Pos(_current_goal)) <= acceptable_range;
    }

private:
    rclcpp_action::Server<ReachGoalAction>::SharedPtr _action_server;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _navsat_sub;
    Pos _robot_pos;
    std::shared_ptr<const ReachGoalAction::Goal> _current_goal;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReachGoalActionServerNode>());
    rclcpp::shutdown();

    return 0;
}