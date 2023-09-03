#pragma once

#include <gmock/gmock.h>

#include "kinematics/i_kinematic_chain.h"
#include "kinematics/joint_state.h"

namespace kinematics
{
    class kinematic_chain_mock : public i_kinematic_chain
    {
    public:
        MOCK_CONST_METHOD0(get_nb_joints, int());
        MOCK_METHOD5(set_state, void(double u1_rad, double u2_rad, double r1_rad, double r2_rad, double p1_meters));
        MOCK_CONST_METHOD0(get_u1_rad, joint_state());
        MOCK_CONST_METHOD0(get_u2_rad, joint_state());
        MOCK_CONST_METHOD0(get_r1_rad, joint_state());
        MOCK_CONST_METHOD0(get_r2_rad, joint_state());
        MOCK_CONST_METHOD0(get_p1_meters, joint_state());
        MOCK_CONST_METHOD0(get_base_link_length_meters, double());
        MOCK_CONST_METHOD0(get_proximal_link_length_meters, double());
        MOCK_CONST_METHOD0(get_distal_link_length_meters, double());
        MOCK_CONST_METHOD0(get_holder_link_length_meters, double());
    };
} // namespace kinematics