#pragma once

#include "kinematics/kinematic_chain.h"

namespace kinematics
{
    class collision_checker
    {
    public:
        collision_checker() = default;
        bool check_collisions(const kinematic_chain &chain);
    };
} // namespace kinematics
