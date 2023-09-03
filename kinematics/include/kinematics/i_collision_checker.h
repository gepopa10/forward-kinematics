#pragma once

#include <memory>

#include "kinematics/i_kinematic_chain.h"

namespace kinematics
{
    class i_collision_checker
    {
    public:
        virtual ~i_collision_checker() = default;
        virtual bool check_collisions(std::shared_ptr<i_kinematic_chain> chain) const = 0;
    };
} // namespace kinematics
