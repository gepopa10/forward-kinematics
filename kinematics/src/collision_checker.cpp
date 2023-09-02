#include <cmath>
#include <iostream>

#include "kinematics/collision_checker.h"
#include "kinematics/kinematic_chain.h"

namespace kinematics
{
    bool collision_checker::check_collisions(const kinematic_chain &chain)
    {
        auto end_z_1 = chain.get_base_link_length_meters() +
                       chain.get_proximal_link_length_meters() * cos(M_PI / 2 + chain.get_u2_rad().get_value());

        auto end_z_2 = end_z_1 + chain.get_distal_link_length_meters() * cos(M_PI / 2 + chain.get_u2_rad().get_value() + chain.get_r1_rad().get_value());

        auto end_z_3 = end_z_2 + chain.get_holder_link_length_meters() *
                                     cos(M_PI / 2 + chain.get_u2_rad().get_value() + chain.get_r1_rad().get_value() + chain.get_r2_rad().get_value());

        auto end_z_4 = end_z_3 + chain.get_p1_meters().get_value() *
                                     cos(M_PI / 2 + chain.get_u2_rad().get_value() + chain.get_r1_rad().get_value() + chain.get_r2_rad().get_value());

        // Check the z-values of these points
        if (end_z_1 < 0 || end_z_2 < 0 || end_z_3 < 0 || end_z_4 < 0)
        {
            std::cout << "Warning: Collision detected!" << std::endl;
            return true; // collision detected
        }

        return false; // no collision
    }
} // namespace kinematics
