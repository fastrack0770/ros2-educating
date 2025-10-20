#include <gtest/gtest.h>

#include "wheeled_model_enhanced/utils.hpp"

TEST(utils, get_vector_3d_abs)
{
    ASSERT_FLOAT_EQ(6.020797289, utils::get_vector_3d_abs(3.2, -5.1, 0));
}

TEST(utils, earth_radius_at)
{
    ASSERT_FLOAT_EQ(6378137.0, utils::earth_radius_at(Pos(Degree(0), Degree(12), 0)));
    // must be 6356752.314, but is not
    ASSERT_FLOAT_EQ(6360159, utils::earth_radius_at(Pos(Degree(66.564056), Degree(12), 122)));
}

TEST(utils, earth_radius_at_2)
{
    ASSERT_FLOAT_EQ(6378137.0, utils::earth_radius_at_2(Pos(Degree(0), Degree(12), 0)));
    // must be 6356752.314, but is not
    ASSERT_FLOAT_EQ(6360135, utils::earth_radius_at_2(Pos(Degree(66.564056), Degree(12), 122)));
}

TEST(utils, distance_in_meters)
{
    // distance must be 376099.61 according to https://gps-coordinates.org/distance-between-coordinates.php
    // difference is not high, so will try to use it
    ASSERT_FLOAT_EQ(376094.41, utils::distance_in_meters(Pos(Degree(35.652832), Degree(139.839478), 0),   // tokyo
                                                         Pos(Degree(35.011665), Degree(135.768326), 0))); // kyoto
}