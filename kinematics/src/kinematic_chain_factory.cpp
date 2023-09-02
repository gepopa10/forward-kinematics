#include <cmath>

#include "kinematics/kinematic_chain_factory.h"
#include "kinematics/kinematic_chain.h"
#include "kinematics/joint_state.h"

namespace kinematics
{
    kinematic_chain kinematic_chain_factory::create() const
    {
        const auto u1 = joint_state(0, -M_PI / 2, M_PI / 2);
        const auto u2 = joint_state(0, -M_PI / 2, M_PI / 2);
        const auto r1 = joint_state(0, -5 * M_PI / 6, 5 * M_PI / 6);
        const auto r2 = joint_state(0, -5 * M_PI / 6, 5 * M_PI / 6);
        const auto p1 = joint_state(0, 0, 0.05);

        kinematic_chain chain(u1, u2, r1, r2, p1, 0.3, 0.4, 0.35, 0.05);

        return chain;
    }
} // namespace kinematics
