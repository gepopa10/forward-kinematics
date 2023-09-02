#include <iostream>
#include <cmath>

#include "kinematics/kinematic_chain_factory.h"
#include "kinematics/forward_kinematics_solver.h"
#include "kinematics/collision_checker.h"

int main() {
    kinematics::kinematic_chain_factory chain_factory;
    auto chain = chain_factory.create();
    auto fk_solver = kinematics::forward_kinematics_solver();
    auto col_checker = kinematics::collision_checker();

    point p = fk_solver.compute_end_effector_position(chain);
    col_checker.check_collisions(chain);

    std::cout << "End effector position (x, y, z): " << p.x << ", " << p.y << ", " << p.z << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    chain.set_state(M_PI/4, -M_PI/2, -M_PI/2, -M_PI/2, 0.06);

    p = fk_solver.compute_end_effector_position(chain);
    col_checker.check_collisions(chain);

    std::cout << "End effector position (x, y, z): " << p.x << ", " << p.y << ", " << p.z << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    chain.set_state(-M_PI/4, M_PI/2, -3*M_PI/4, M_PI/4, 0);

    p = fk_solver.compute_end_effector_position(chain);
    col_checker.check_collisions(chain);

    std::cout << "End effector position (x, y, z): " << p.x << ", " << p.y << ", " << p.z << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    chain.set_state(M_PI/2, M_PI/4, -M_PI/6, 0, 0.05);

    p = fk_solver.compute_end_effector_position(chain);
    col_checker.check_collisions(chain);

    std::cout << "End effector position (x, y, z): " << p.x << ", " << p.y << ", " << p.z << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    chain.set_state(-M_PI/4, 40*M_PI/180, 140*M_PI/180, 0, 0.05);

    p = fk_solver.compute_end_effector_position(chain);
    col_checker.check_collisions(chain);

    std::cout << "End effector position (x, y, z): " << p.x << ", " << p.y << ", " << p.z << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    return 0;
}
