#include <gtest/gtest.h>

#include "wheeled_model_enhanced/types/cartesian.hpp"

TEST(types, cartesian)
{
    // Cartesian operator-(Cartesian lhv, const Cartesian &rhv)
    {
        Cartesian lhv(5, 7, 9);
        Cartesian rhv(2, 3, 4);
        EXPECT_EQ(Cartesian(3, 4, 5), lhv - rhv);
        EXPECT_EQ(Cartesian(5, 7, 9), lhv) << lhv;
    }
    // Cartesian operator+(Cartesian lhv, const Cartesian &rhv)
    {
        Cartesian lhv(5, 7, 9);
        Cartesian rhv(2, 3, 4);
        EXPECT_EQ(Cartesian(7, 10, 13), lhv + rhv);
        EXPECT_EQ(Cartesian(5, 7, 9), lhv) << lhv;
    }
}