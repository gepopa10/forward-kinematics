#include <cmath>
#include <iostream>

#include "kinematics/forward_kinematics_solver.h"
#include "kinematics/kinematic_chain.h"
#include "kinematics/point.h"

namespace kinematics
{
    point forward_kinematics_solver::compute_end_effector_position(const kinematic_chain &chain) const
    {
        const auto zx_plane_position = chain.get_proximal_link_length_meters() * sin(M_PI / 2 + chain.get_u2_rad().get_value()) +
                                       chain.get_distal_link_length_meters() * sin(M_PI / 2 + chain.get_u2_rad().get_value() + chain.get_r1_rad().get_value()) +
                                       (chain.get_holder_link_length_meters() + chain.get_p1_meters().get_value()) *
                                           sin(M_PI / 2 + chain.get_u2_rad().get_value() + chain.get_r1_rad().get_value() + chain.get_r2_rad().get_value());

        double x = cos(chain.get_u1_rad().get_value()) * zx_plane_position;
        double y = sin(chain.get_u1_rad().get_value()) * zx_plane_position;
        double z = chain.get_base_link_length_meters() +
                   chain.get_proximal_link_length_meters() * cos(M_PI / 2 + chain.get_u2_rad().get_value()) +
                   chain.get_distal_link_length_meters() * cos(M_PI / 2 + chain.get_u2_rad().get_value() + chain.get_r1_rad().get_value()) +
                   (chain.get_holder_link_length_meters() + chain.get_p1_meters().get_value()) *
                       cos(M_PI / 2 + chain.get_u2_rad().get_value() + chain.get_r1_rad().get_value() + chain.get_r2_rad().get_value());

        return {x, y, z};
    }
} // namespace kinematics
