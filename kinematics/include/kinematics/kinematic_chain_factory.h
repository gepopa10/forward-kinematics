#pragma once

#include "kinematics/kinematic_chain.h"

namespace kinematics
{
    class kinematic_chain_factory
    {
    public:
        kinematic_chain_factory() = default;
        kinematic_chain create() const;
    };
} // namespace kinematics
