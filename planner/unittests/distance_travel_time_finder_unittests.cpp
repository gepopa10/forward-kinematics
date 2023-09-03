#include <gtest/gtest.h>

#include "planner/distance_travel_time_finder.h"

namespace planner
{
    struct GIVEN_distance_travel_time_finder : public ::testing::Test
    {
    protected:
        distance_travel_time_finder _distance_travel_time_finder;

        GIVEN_distance_travel_time_finder() : _distance_travel_time_finder() {}
    };

    struct GIVEN_max_speed_reached : public GIVEN_distance_travel_time_finder
    {
    protected:
        const double _expected_time = 1.5;
        const double _start = 0;
        const double _end = 10;
        const double _max_vel = 10;
        const double _max_acc = 20;
    };

    TEST_F(GIVEN_max_speed_reached, WHEN_compute_time_of_travel_secs_THEN_return_expected_time)
    {
        auto res = _distance_travel_time_finder.compute_time_of_travel_secs(_start, _end, _max_vel, _max_acc);
        EXPECT_EQ(_expected_time, res);
    }

    struct GIVEN_max_speed_not_reached : public GIVEN_distance_travel_time_finder
    {
    protected:
        const double _expected_time = 8;
        const double _start = 0;
        const double _end = 16;
        const double _max_vel = 10;
        const double _max_acc = 1;
    };

    TEST_F(GIVEN_max_speed_not_reached, WHEN_compute_time_of_travel_secs_THEN_return_expected_time)
    {
        auto res = _distance_travel_time_finder.compute_time_of_travel_secs(_start, _end, _max_vel, _max_acc);
        EXPECT_EQ(_expected_time, res);
    }

} // namespace planner