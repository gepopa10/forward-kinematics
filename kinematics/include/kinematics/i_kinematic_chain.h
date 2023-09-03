#pragma once

#include "kinematics/joint_state.h"

namespace kinematics
{
    class i_kinematic_chain
    {
    public:
        virtual ~i_kinematic_chain() = default;
        virtual int get_nb_joints() const = 0;
        virtual void set_state(double u1_rad, double u2_rad, double r1_rad, double r2_rad, double p1_meters) = 0;
        virtual joint_state get_u1_rad() const = 0;
        virtual joint_state get_u2_rad() const = 0;
        virtual joint_state get_r1_rad() const = 0;
        virtual joint_state get_r2_rad() const = 0;
        virtual joint_state get_p1_meters() const = 0;
        virtual double get_base_link_length_meters() const = 0;
        virtual double get_proximal_link_length_meters() const = 0;
        virtual double get_distal_link_length_meters() const = 0;
        virtual double get_holder_link_length_meters() const = 0;
    };
} // namespace kinematics
