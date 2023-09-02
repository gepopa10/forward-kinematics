#pragma once

#include <vector>

#include "kinematics/joint_state.h"

namespace kinematics
{
    struct kinematic_chain
    {
        kinematic_chain() = delete;
        kinematic_chain(joint_state u1_rad, joint_state u2_rad, joint_state r1_rad, joint_state r2_rad, joint_state p1_meters,
                        double base_link_length_meters, double proximal_link_length_meters, double distal_link_length_meters, double holder_link_length_meters) : _u1_rad(u1_rad), _u2_rad(u2_rad), _r1_rad(r1_rad), _r2_rad(r2_rad), _p1_meters(p1_meters), _base_link_length_meters(base_link_length_meters),
                                                                                                                                                                  _proximal_link_length_meters(proximal_link_length_meters), _distal_link_length_meters(distal_link_length_meters), _holder_link_length_meters(holder_link_length_meters)
        {
        }

        joint_state get_u1_rad() const { return _u1_rad; };
        joint_state get_u2_rad() const { return _u2_rad; };
        joint_state get_r1_rad() const { return _r1_rad; };
        joint_state get_r2_rad() const { return _r2_rad; };
        joint_state get_p1_meters() const { return _p1_meters; };

        double get_base_link_length_meters() const { return _base_link_length_meters; };
        double get_proximal_link_length_meters() const { return _proximal_link_length_meters; };
        double get_distal_link_length_meters() const { return _distal_link_length_meters; };
        double get_holder_link_length_meters() const { return _holder_link_length_meters; };
        void set_state(double u1_rad, double u2_rad, double r1_rad, double r2_rad, double p1_meters)
        {
            _u1_rad.set_value(u1_rad);
            _u2_rad.set_value(u2_rad);
            _r1_rad.set_value(r1_rad);
            _r2_rad.set_value(r2_rad);
            _p1_meters.set_value(p1_meters);
        }

    private:
        joint_state _u1_rad;
        joint_state _u2_rad;
        joint_state _r1_rad;
        joint_state _r2_rad;
        joint_state _p1_meters;

        double _base_link_length_meters;
        double _proximal_link_length_meters;
        double _distal_link_length_meters;
        double _holder_link_length_meters;
    };
} // namespace kinematics
