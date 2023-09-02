#include <gtest/gtest.h>
#include <cmath>

#include "kinematics/forward_kinematics_solver.h"
#include "kinematics/kinematic_chain_factory.h"
#include "kinematics/point.h"

namespace kinematics
{
    double epsilon = 1e-5;

    struct fk_GIVEN_kinematic_chain : public ::testing::Test
    {
    protected:
        kinematic_chain_factory _chain_factory;
        kinematic_chain _chain;
        forward_kinematics_solver _fk_solver;

        fk_GIVEN_kinematic_chain() : _chain(_chain_factory.create()), _fk_solver() {}
    };

    struct fk_GIVEN_case_1 : public fk_GIVEN_kinematic_chain
    {
    protected:
        const point _expected_point = point(0.8, 0.0, 0.3);
    };

    TEST_F(fk_GIVEN_case_1, WHEN_compute_end_effector_position_THEN_return_expected_point)
    {
        EXPECT_EQ(_expected_point, _fk_solver.compute_end_effector_position(_chain));
    }

    struct fk_GIVEN_case_2 : public fk_GIVEN_kinematic_chain
    {
    protected:
        const point _expected_point = point(-0.247487, -0.247487, 0.6);

        fk_GIVEN_case_2()
        {
            _chain.set_state(M_PI / 4, -M_PI / 2, -M_PI / 2, -M_PI / 2, 0.06);
        }
    };

    TEST_F(fk_GIVEN_case_2, WHEN_compute_end_effector_position_THEN_return_expected_point)
    {
        const point result = _fk_solver.compute_end_effector_position(_chain);

        EXPECT_NEAR(_expected_point.x, result.x, epsilon);
        EXPECT_NEAR(_expected_point.y, result.y, epsilon);
        EXPECT_NEAR(_expected_point.z, result.z, epsilon);
    }

    struct fk_GIVEN_case_3 : public fk_GIVEN_kinematic_chain
    {
    protected:
        const point _expected_point = point(0.210355, -0.210355, 0.147487);

        fk_GIVEN_case_3()
        {
            _chain.set_state(-M_PI / 4, M_PI / 2, -3 * M_PI / 4, M_PI / 4, 0);
        }
    };

    TEST_F(fk_GIVEN_case_3, WHEN_compute_end_effector_position_THEN_return_expected_point)
    {
        const point result = _fk_solver.compute_end_effector_position(_chain);

        EXPECT_NEAR(_expected_point.x, result.x, epsilon);
        EXPECT_NEAR(_expected_point.y, result.y, epsilon);
        EXPECT_NEAR(_expected_point.z, result.z, epsilon);
    }

    struct fk_GIVEN_case_4 : public fk_GIVEN_kinematic_chain
    {
    protected:
        const point _expected_point = point(0, 0.717509, -0.0993113);

        fk_GIVEN_case_4()
        {
            _chain.set_state(M_PI / 2, M_PI / 4, -M_PI / 6, 0, 0.05);
        }
    };

    TEST_F(fk_GIVEN_case_4, WHEN_compute_end_effector_position_THEN_return_expected_point)
    {
        const point result = _fk_solver.compute_end_effector_position(_chain);

        EXPECT_NEAR(_expected_point.x, result.x, epsilon);
        EXPECT_NEAR(_expected_point.y, result.y, epsilon);
        EXPECT_NEAR(_expected_point.z, result.z, epsilon);
    }

    struct fk_GIVEN_case_5 : public fk_GIVEN_kinematic_chain
    {
    protected:
        const point _expected_point = point(-0.101528, 0.101528, 0.042885);

        fk_GIVEN_case_5()
        {
            _chain.set_state(-M_PI / 4, 40 * M_PI / 180, 140 * M_PI / 180, 0, 0.05);
        }
    };

    TEST_F(fk_GIVEN_case_5, WHEN_compute_end_effector_position_THEN_return_expected_point)
    {
        const point result = _fk_solver.compute_end_effector_position(_chain);

        EXPECT_NEAR(_expected_point.x, result.x, epsilon);
        EXPECT_NEAR(_expected_point.y, result.y, epsilon);
        EXPECT_NEAR(_expected_point.z, result.z, epsilon);
    }

} // namespace kinematics