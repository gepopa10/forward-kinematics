#pragma once

#include <memory>

#include "kinematics/i_kinematic_chain.h"

namespace kinematics
{
    class kinematic_chain_factory
    {
    public:
        kinematic_chain_factory() = default;
        std::shared_ptr<i_kinematic_chain> create() const;
    };
} // namespace kinematics
