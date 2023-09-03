#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "kinematics/collision_checker.h"
#include "kinematics/i_kinematic_chain.h"
#include "kinematics/kinematic_chain_factory.h"

namespace kinematics
{
    struct collision_checker_GIVEN_kinematic_chain : public ::testing::Test
    {
    protected:
        kinematic_chain_factory _chain_factory;
        std::shared_ptr<i_kinematic_chain> _chain;
        collision_checker _collision_checker;

        collision_checker_GIVEN_kinematic_chain() : _collision_checker()
        {
            _chain = _chain_factory.create();
        }
    };

    struct collision_checker_GIVEN_case_1 : public collision_checker_GIVEN_kinematic_chain
    {
    protected:
        const bool _expected_collision = false;
    };

    TEST_F(collision_checker_GIVEN_case_1, WHEN_check_collisions_THEN_return_expected_collision)
    {
        auto res = _collision_checker.check_collisions(_chain);
        EXPECT_EQ(_expected_collision, res);
    }

    struct collision_checker_GIVEN_case_2 : public collision_checker_GIVEN_kinematic_chain
    {
    protected:
        const bool _expected_collision = false;

        collision_checker_GIVEN_case_2()
        {
            _chain->set_state(M_PI / 4, -M_PI / 2, -M_PI / 2, -M_PI / 2, 0.06);
        }
    };

    TEST_F(collision_checker_GIVEN_case_2, WHEN_check_collisions_THEN_return_expected_collision)
    {
        EXPECT_EQ(_expected_collision, _collision_checker.check_collisions(_chain));
    }

    struct collision_checker_GIVEN_case_3 : public collision_checker_GIVEN_kinematic_chain
    {
    protected:
        const bool _expected_collision = true;

        collision_checker_GIVEN_case_3()
        {
            _chain->set_state(-M_PI / 4, M_PI / 2, -3 * M_PI / 4, M_PI / 4, 0);
        }
    };

    TEST_F(collision_checker_GIVEN_case_3, WHEN_check_collisions_THEN_return_expected_collision)
    {
        EXPECT_EQ(_expected_collision, _collision_checker.check_collisions(_chain));
    }

    struct collision_checker_GIVEN_case_4 : public collision_checker_GIVEN_kinematic_chain
    {
    protected:
        const bool _expected_collision = true;

        collision_checker_GIVEN_case_4()
        {
            _chain->set_state(M_PI / 2, M_PI / 4, -M_PI / 6, 0, 0.05);
        }
    };

    TEST_F(collision_checker_GIVEN_case_4, WHEN_check_collisions_THEN_return_expected_collision)
    {
        EXPECT_EQ(_expected_collision, _collision_checker.check_collisions(_chain));
    }

    struct collision_checker_GIVEN_case_5 : public collision_checker_GIVEN_kinematic_chain
    {
    protected:
        const bool _expected_collision = false;

        collision_checker_GIVEN_case_5()
        {
            _chain->set_state(-M_PI / 4, 40 * M_PI / 180, 140 * M_PI / 180, 0, 0.05);
        }
    };

    TEST_F(collision_checker_GIVEN_case_5, WHEN_check_collisions_THEN_return_expected_collision)
    {
        EXPECT_EQ(_expected_collision, _collision_checker.check_collisions(_chain));
    }

} // namespace kinematics