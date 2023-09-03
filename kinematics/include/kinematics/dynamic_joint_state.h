#pragma once

#include <iostream>
#include <algorithm>

#include "kinematics/joint_state.h"

namespace kinematics
{
    struct dynamic_joint_state : public joint_state
    {
        dynamic_joint_state() = delete;
        dynamic_joint_state(double value, double min, double max, double max_vel, double max_acc) : joint_state(value, min, max), _max_vel(max_vel), _max_acc(max_acc)
        {
        }
        double get_max_vel() const { return _max_vel; };
        double get_max_acc() const { return _max_acc; };
    private:
        double _max_vel;
        double _max_acc;
    };
} // namespace kinematics
