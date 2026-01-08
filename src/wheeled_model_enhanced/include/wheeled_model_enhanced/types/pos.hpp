#pragma once

#include "wheeled_model_enhanced/action/reach_goal.hpp"
#include "wheeled_model_enhanced/types/meter.hpp"
#include "wheeled_model_enhanced/types/optional.hpp"
#include "wheeled_model_enhanced/types/radian.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"

/**
 * Pos
 * GPS position holder
 */
class Pos
{
  private:
    using ReachGoalAction = wheeled_model_enhanced::action::ReachGoal;

  public:
    constexpr Pos()
    {
    }

    constexpr Pos(Radian latitude, Radian longitude, Meter altitude)
        : _latitude(latitude), _longitude(longitude), _altitude(altitude)
    {
    }

    Pos(std::shared_ptr<const ReachGoalAction::Goal> goal)
        : _latitude(goal->goal_lat), _longitude(goal->goal_long), _altitude(0.f)
    {
    }
    Pos(const sensor_msgs::msg::NavSatFix &msg)
        : _latitude(Degree(msg.latitude)), _longitude(Degree(msg.longitude)), _altitude(msg.altitude)
    {
    }
    constexpr Radian latitude() const
    {
        return _latitude;
    }

    constexpr Radian longitude() const
    {
        return _longitude;
    }

    constexpr Meter altitude() const
    {
        return _altitude;
    }

    void setLatitude(Radian new_lat)
    {
        _latitude = new_lat;
    }

    void setLongitude(Radian new_long)
    {
        _longitude = new_long;
    }

    void setAltitude(Meter new_alt)
    {
        _altitude = new_alt;
    }

    Pos &operator=(const sensor_msgs::msg::NavSatFix &msg)
    {
        _latitude = Radian(Degree(msg.latitude));
        _longitude = Radian(Degree(msg.longitude));
        _altitude = msg.altitude;
        return *this;
    }

    constexpr bool operator==(const Pos &rhv) const noexcept
    {
        return std::tie(_latitude, _longitude, _altitude) == std::tie(rhv._latitude, rhv._longitude, rhv._altitude);
    }

    constexpr bool operator!=(const Pos &rhv) const noexcept
    {
        return !(*this == rhv);
    }

    friend std::ostream &operator<<(std::ostream &os, const Pos &rhv)
    {
        return os << "{ lat: " << rhv._latitude << " long: " << rhv._longitude << " alt: " << rhv._altitude << "}";
    }

  private:
    Radian _latitude {0.f};
    Radian _longitude {0.f};
    Meter _altitude {0.f};
};