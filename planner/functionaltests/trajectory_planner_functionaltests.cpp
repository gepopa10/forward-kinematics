#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numeric>

#include "kinematics/kinematic_chain_factory.h"
#include "kinematics/collision_checker.h"
#include "kinematics/dynamic_joint_state.h"
#include "planner/trajectory_planner.h"
#include "planner/distance_travel_time_finder.h"

namespace planner
{
    bool are_transposed_equal(const std::vector<std::vector<kinematics::dynamic_joint_state>> &a, const std::vector<std::vector<double>> &b)
    {
        for (size_t i = 0; i < a.size(); ++i)
        {
            for (size_t j = 0; j < a[i].size(); ++j)
            {
                if (a.at(i).at(j).get_value() != b.at(j).at(i))
                {
                    return false;
                }
            }
        }
        return true;
    }

    struct functionaltests_GIVEN_trajectory_planner : public ::testing::Test
    {
    protected:
        kinematics::collision_checker _collision_checker;
        kinematics::kinematic_chain_factory _chain_factory;
        std::shared_ptr<kinematics::i_kinematic_chain> _chain;
        distance_travel_time_finder _distance_travel_time_finder;
        std::unique_ptr<trajectory_planner> _trajectory_planner;

        functionaltests_GIVEN_trajectory_planner()
        {
            _chain = _chain_factory.create();
            _trajectory_planner = std::make_unique<trajectory_planner>(_distance_travel_time_finder,
                                                                       _collision_checker,
                                                                       _chain);
        }
    };

    struct GIVEN_joint_1_slower_than_joint_2 : public functionaltests_GIVEN_trajectory_planner
    {
    protected:
        const double _freq = 4;
        const double speed_joint_1 = 0.5; // should take 2 secs since this is limiting joint
        const double speed_joint_2 = 1;
        const double acc_joint_1 = std::numeric_limits<double>::max();
        const double acc_joint_2 = std::numeric_limits<double>::max();
        const kinematics::dynamic_joint_state _stationary_joint = kinematics::dynamic_joint_state(0, 0, 0, 0, 0);
        const std::vector<kinematics::dynamic_joint_state> _start = {
            kinematics::dynamic_joint_state(0, -M_PI / 2, M_PI / 2, speed_joint_1, acc_joint_1),
            kinematics::dynamic_joint_state(0, -M_PI / 2, M_PI / 2, speed_joint_2, acc_joint_2),
            _stationary_joint,
            _stationary_joint,
            _stationary_joint};

        const std::vector<kinematics::dynamic_joint_state> _end = {
            kinematics::dynamic_joint_state(1, -M_PI / 2, M_PI / 2, speed_joint_1, acc_joint_1),
            kinematics::dynamic_joint_state(1, -M_PI / 2, M_PI / 2, speed_joint_2, acc_joint_2),
            _stationary_joint,
            _stationary_joint,
            _stationary_joint};

        const std::vector<double> _expected_joint_positions = {0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1};
        const std::vector<double> _expected_stationary_positions = std::vector<double>(_expected_joint_positions.size(), 0);
        std::vector<std::vector<double>> _expected_trajectory = {
            _expected_joint_positions,
            _expected_joint_positions,
            _expected_stationary_positions,
            _expected_stationary_positions,
            _expected_stationary_positions};
    };

    TEST_F(GIVEN_joint_1_slower_than_joint_2, WHEN_plan_THEN_return_expected_result)
    {
        const auto res = _trajectory_planner->plan(_start, _end, _freq);
        EXPECT_TRUE(are_transposed_equal(res, _expected_trajectory));
    }
} // namespace planner