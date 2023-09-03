#pragma once

#include <memory>

#include "kinematics/i_kinematic_chain.h"
#include "kinematics/i_collision_checker.h"

namespace kinematics
{
    class collision_checker : public i_collision_checker
    {
    public:
        collision_checker() = default;
        virtual bool check_collisions(std::shared_ptr<i_kinematic_chain> chain) const override;
    };
} // namespace kinematics
