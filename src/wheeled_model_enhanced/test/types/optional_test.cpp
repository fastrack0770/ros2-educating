#include <gtest/gtest.h>

#include "wheeled_model_enhanced/types/optional.hpp"

struct OptionalTest
{
    OptionalTest()
    {
    }
    OptionalTest(bool ia, bool ib) : a(ia), b(ib)
    {
    }
    bool a;
    bool b;
};

struct OptionalDestructorTest
{
    OptionalDestructorTest(std::function<void()> callback) : _callback(callback)
    {
    }
    ~OptionalDestructorTest()
    {
        _callback();
    }
    std::function<void()> _callback;
};

TEST(types, optional)
{
    // constructors
    {
        Optional<int> opt(3);

        EXPECT_TRUE(opt.has_value());
        EXPECT_EQ(3, *opt);
    }
    {
        Optional<OptionalTest> opt({true, false});

        EXPECT_TRUE(opt.has_value());
        EXPECT_TRUE(opt->a);
        EXPECT_FALSE(opt->b);
    }
    {
        Optional<OptionalTest> opt;

        EXPECT_FALSE(opt.has_value());
        EXPECT_NO_THROW(opt->a);
        EXPECT_NO_THROW(opt->b);
    }
    {
        Optional<int> opt;

        EXPECT_FALSE(opt.has_value());
        EXPECT_NO_THROW(*opt);
    }
    {
        const Optional<int> opt(5);
        Optional<int> opt_copy(opt);

        EXPECT_TRUE(opt.has_value());
        EXPECT_EQ(5, *opt);
        EXPECT_TRUE(opt_copy.has_value());
        EXPECT_EQ(5, *opt_copy);
    }
    {
        const Optional<int> opt;
        Optional<int> opt_copy(opt);

        EXPECT_FALSE(opt.has_value());
        EXPECT_FALSE(opt_copy.has_value());
    }
    {
        Optional<int> opt(5);
        Optional<int> opt_copy(std::move(opt));

        EXPECT_FALSE(opt.has_value());
        EXPECT_TRUE(opt_copy.has_value());
        EXPECT_EQ(5, *opt_copy);
    }
    // Optional<T> &operator=(const T &rhv)
    {
        Optional<int> opt;
        int rhv = 3;
        opt = rhv;

        EXPECT_TRUE(opt.has_value());
        EXPECT_EQ(3, *opt);
    }
    // constexpr Optional &operator=(const Optional &rhv)
    {
        Optional<int> opt(2);
        Optional<int> rhv(7);

        EXPECT_EQ(2, *opt);

        opt = rhv;

        EXPECT_EQ(7, *rhv);
        EXPECT_EQ(7, *opt);
    }
    // void reset()
    {
        bool destructor_was_called = false;
        const auto callback = [&destructor_was_called]() { destructor_was_called = true; };
        Optional<OptionalDestructorTest> opt({callback});
        opt.reset();

        EXPECT_FALSE(opt.has_value());
        EXPECT_TRUE(destructor_was_called);
    }
    // constexpr T &value()
    {
        constexpr Optional<int> opt;
        EXPECT_THROW(opt.value(), BadOptionalAccess);
    }
    {
        Optional<int> opt;
        EXPECT_THROW(opt.value(), BadOptionalAccess);

        opt = 7;
        EXPECT_EQ(7, opt.value());
    }
    // constexpr const T &value() const
    {
        constexpr Optional<int> opt;
        EXPECT_THROW(opt.value(), BadOptionalAccess);
    }
    {
        constexpr Optional<int> opt(7);
        EXPECT_EQ(7, opt.value());
    }
}