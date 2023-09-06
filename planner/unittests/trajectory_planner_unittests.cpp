#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <vector>
#include <memory>

#include "planner/trajectory_planner.h"
#include "kinematics/kinematic_chain.h"
#include "kinematics/dynamic_joint_state.h"
#include "planner/travel_time_finder_mock.h"
#include "kinematics/collision_checker_mock.h"
#include "kinematics/kinematic_chain_mock.h"

using ::testing::Return;

namespace planner
{
    struct GIVEN_trajectory_planner : public ::testing::Test
    {
    protected:
        kinematics::collision_checker_mock _collision_checker_mock;
        travel_time_finder_mock _travel_time_finder_mock;
        std::shared_ptr<kinematics::kinematic_chain_mock> _kinematic_chain_mock;
        std::unique_ptr<trajectory_planner> _trajectory_planner;

        GIVEN_trajectory_planner()
        {
            _kinematic_chain_mock = std::make_shared<kinematics::kinematic_chain_mock>();
            _trajectory_planner = std::make_unique<trajectory_planner>(_travel_time_finder_mock,
                                                                       _collision_checker_mock,
                                                                       _kinematic_chain_mock);
            ON_CALL(*_kinematic_chain_mock, get_nb_joints()).WillByDefault(Return(1));
        }
    };

    struct GIVEN_start_and_end_joint_size_mismatch : public GIVEN_trajectory_planner
    {
    protected:
        std::vector<kinematics::dynamic_joint_state> _start = {kinematics::dynamic_joint_state(0, 0, 0, 0, 0)};
        std::vector<kinematics::dynamic_joint_state> _end = {kinematics::dynamic_joint_state(0, 0, 0, 0, 0),
                                                             kinematics::dynamic_joint_state(0, 0, 0, 0, 0)};
        double _freq = 1;
    };

    TEST_F(GIVEN_start_and_end_joint_size_mismatch, WHEN_plan_THEN_throw_invalid_argument)
    {
        EXPECT_THROW(_trajectory_planner->plan(_start, _end, _freq), std::invalid_argument);
    }

    struct GIVEN_start_and_chain_joint_size_mismatch : public GIVEN_start_and_end_joint_size_mismatch
    {
    protected:
        std::vector<kinematics::dynamic_joint_state> _end = _start;
        GIVEN_start_and_chain_joint_size_mismatch()
        {
            ON_CALL(*_kinematic_chain_mock, get_nb_joints()).WillByDefault(Return(3));
        }
    };

    TEST_F(GIVEN_start_and_chain_joint_size_mismatch, WHEN_plan_THEN_throw_invalid_argument)
    {
        EXPECT_THROW(_trajectory_planner->plan(_start, _end, _freq), std::invalid_argument);
    }

    struct GIVEN_invalid_frequency : public GIVEN_start_and_end_joint_size_mismatch
    {
    protected:
        std::vector<kinematics::dynamic_joint_state> _end = _start;
        const int frequency = 0;
    };

    TEST_F(GIVEN_invalid_frequency, WHEN_plan_THEN_throw_invalid_argument)
    {
        EXPECT_THROW(_trajectory_planner->plan(_start, _end, frequency), std::invalid_argument);
    }

} // namespace planner