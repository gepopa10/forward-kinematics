#pragma once

#include <vector>
#include <memory>

#include "planner/i_travel_time_finder.h"
#include "kinematics/dynamic_joint_state.h"
#include "kinematics/i_collision_checker.h"
#include "kinematics/kinematic_chain.h"

namespace planner
{
    class trajectory_planner
    {
    public:
        trajectory_planner(const i_travel_time_finder &travel_time_finder, const kinematics::i_collision_checker &collision_checker, std::shared_ptr<kinematics::i_kinematic_chain> chain);
        std::vector<std::vector<kinematics::dynamic_joint_state>> plan(const std::vector<kinematics::dynamic_joint_state>& start, const std::vector<kinematics::dynamic_joint_state>& end, double frequency);

    private:
        const i_travel_time_finder &_travel_time_finder;
        const kinematics::i_collision_checker &_collision_checker;
        std::shared_ptr<kinematics::i_kinematic_chain> _chain;
    };
} // namespace planner
