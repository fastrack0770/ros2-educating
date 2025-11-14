#pragma once

#include "wheeled_model_enhanced/types/degree.hpp"
#include "wheeled_model_enhanced/types/radian.hpp"
#include "wheeled_model_enhanced/types/meter.hpp"
#include "wheeled_model_enhanced/types/kilometer.hpp"
#include "wheeled_model_enhanced/types/cartesian.hpp"
#include "wheeled_model_enhanced/types/optional.hpp"
#include "wheeled_model_enhanced/types/plane.hpp"
#include "wheeled_model_enhanced/types/pos.hpp"
#include "wheeled_model_enhanced/types/vector3d.hpp"


inline double operator-(builtin_interfaces::msg::Time lhv, const builtin_interfaces::msg::Time &rhv)
{
    const auto passed_time = lhv.sec - rhv.sec + (static_cast<double>(lhv.nanosec) - rhv.nanosec) / 1'000'000'000;

    return passed_time;
}
