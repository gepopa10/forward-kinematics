#include <gtest/gtest.h>
#include <cmath>

#include "kinematics/collision_checker.h"
#include "kinematics/kinematic_chain_factory.h"

namespace kinematics
{
    struct collision_checker_GIVEN_kinematic_chain : public ::testing::Test
    {
    protected:
        kinematic_chain_factory _chain_factory;
        kinematic_chain _chain;
        collision_checker _collision_checker;

        collision_checker_GIVEN_kinematic_chain() : _chain(_chain_factory.create()), _collision_checker() {}
    };

    struct collision_checker_GIVEN_case_1 : public collision_checker_GIVEN_kinematic_chain
    {
    protected:
        const bool _expected_collision = false;
    };

    TEST_F(collision_checker_GIVEN_case_1, WHEN_check_collisions_THEN_return_expected_collision)
    {
        std::cout <<"before" << std::endl;
        auto res = _collision_checker.check_collisions(_chain);
        std::cout <<"after" << std::endl;
        EXPECT_EQ(_expected_collision, res);
    }

    struct collision_checker_GIVEN_case_2 : public collision_checker_GIVEN_kinematic_chain
    {
    protected:
        const bool _expected_collision = false;

        collision_checker_GIVEN_case_2()
        {
            _chain.set_state(M_PI / 4, -M_PI / 2, -M_PI / 2, -M_PI / 2, 0.06);
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
            _chain.set_state(-M_PI / 4, M_PI / 2, -3 * M_PI / 4, M_PI / 4, 0);
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
            _chain.set_state(M_PI / 2, M_PI / 4, -M_PI / 6, 0, 0.05);
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
            _chain.set_state(-M_PI / 4, 40 * M_PI / 180, 140 * M_PI / 180, 0, 0.05);
        }
    };

    TEST_F(collision_checker_GIVEN_case_5, WHEN_check_collisions_THEN_return_expected_collision)
    {
        EXPECT_EQ(_expected_collision, _collision_checker.check_collisions(_chain));
    }

} // namespace kinematics