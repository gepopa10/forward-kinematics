#pragma once

#include "kinematics/kinematic_chain.h"
#include "kinematics/point.h"

namespace kinematics
{
    class forward_kinematics_solver
    {
    public:
        forward_kinematics_solver() = default;
        point compute_end_effector_position(const kinematic_chain &chain) const;
    };
} // namespace kinematics
