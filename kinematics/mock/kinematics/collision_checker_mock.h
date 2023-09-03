#pragma once

#include <gmock/gmock.h>

#include "kinematics/i_collision_checker.h"
#include "kinematics/i_kinematic_chain.h"

namespace kinematics
{
    class collision_checker_mock : public i_collision_checker
    {
    public:
        MOCK_CONST_METHOD1(check_collisions, bool(std::shared_ptr<i_kinematic_chain> chain));
    };
} // namespace kinematics