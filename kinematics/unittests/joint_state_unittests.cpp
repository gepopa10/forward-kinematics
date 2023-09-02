#include <gtest/gtest.h>

#include "kinematics/joint_state.h"

namespace kinematics
{

    struct GIVEN_joint_state_over_max : public ::testing::Test
    {
    protected:
        joint_state _s = joint_state(0.5, -0.2, 0.2);
        double _expected_value = 0.2;
    };

    TEST_F(GIVEN_joint_state_over_max, WHEN_get_value_THEN_return_expected_value)
    {
        EXPECT_EQ(_expected_value, _s.get_value());
    }

} // namespace kinematics