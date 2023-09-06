#include <cmath>
#include <string>

#include "planner/trajectory_planner.h"

namespace planner
{
    trajectory_planner::trajectory_planner(const i_travel_time_finder &travel_time_finder,
                                           const kinematics::i_collision_checker &collision_checker,
                                           std::shared_ptr<kinematics::i_kinematic_chain> chain)
        : _travel_time_finder(travel_time_finder),
          _collision_checker(collision_checker),
          _chain(chain)
    {
    }

    std::vector<std::vector<kinematics::dynamic_joint_state>> trajectory_planner::plan(
        const std::vector<kinematics::dynamic_joint_state>& start,
        const std::vector<kinematics::dynamic_joint_state>& end,
        double frequency)
    {
        if (start.size() != end.size())
        {
            throw std::invalid_argument("start joint size of: " + std::to_string(start.size()) + " does not match end joint size of: " + std::to_string(end.size()));
        }

        if (start.size() != _chain->get_nb_joints())
        {
            throw std::invalid_argument("start joint size of: " + std::to_string(start.size()) + " does not match chain nb of joints: " + std::to_string(_chain->get_nb_joints()));
        }

        if (frequency <=0)
        {
            throw std::invalid_argument("frequency of: " + std::to_string(frequency) + " is invalid");
        }

        double max_time = 0;

        const size_t nb_joints = start.size();

        for (int i = 0; i < nb_joints; ++i)
        {
            const double start_pos = start[i].get_value();
            const double end_pos = end[i].get_value();
            const double max_vel = start[i].get_max_vel();
            const double max_acc = start[i].get_max_acc();
            max_time = std::max(max_time, _travel_time_finder.compute_time_of_travel_secs(start_pos, end_pos, max_vel, max_acc));
        }

        std::vector<std::vector<kinematics::dynamic_joint_state>> trajectory;

        size_t total_steps = static_cast<size_t>(std::ceil(max_time * frequency));

        for (size_t step = 0; step < total_steps + 1; ++step)
        {
            double current_time = step * (1.0 / frequency);
            std::vector<kinematics::dynamic_joint_state> joint_states_at_this_step;

            for (int j = 0; j < nb_joints; ++j)
            {
                double start_pos = start[j].get_value();
                double end_pos = end[j].get_value();

                double interpolated_pos = start_pos + (end_pos - start_pos) * (current_time / max_time);
                const auto interpolated_dynamic_joint_state = kinematics::dynamic_joint_state(interpolated_pos, start[j].get_min(), start[j].get_max(), start[j].get_max_vel(), start[j].get_max_acc());

                joint_states_at_this_step.push_back(interpolated_dynamic_joint_state);
            }

            _chain->set_state(joint_states_at_this_step.at(0).get_value(),
            joint_states_at_this_step.at(1).get_value(),
            joint_states_at_this_step.at(2).get_value(),
            joint_states_at_this_step.at(3).get_value(),
            joint_states_at_this_step.at(4).get_value());

            _collision_checker.check_collisions(_chain);

            trajectory.push_back(joint_states_at_this_step);
        }

        return trajectory;
    }
} // namespace planner
