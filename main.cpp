#include <iostream>
#include <cmath>

#include "kinematics/kinematic_chain_factory.h"
#include "kinematics/forward_kinematics_solver.h"
#include "kinematics/collision_checker.h"
#include "kinematics/dynamic_joint_state.h"
#include "planner/trajectory_planner.h"
#include "planner/distance_travel_time_finder.h"

void print_trajectory(const std::vector<std::vector<kinematics::dynamic_joint_state>> traj)
{
    for (int i = 0; i < traj.size(); i++)
    {
        std::cout << i << " : ";
        for (const auto &joint : traj[i])
        {
            std::cout << joint.get_value() << ", ";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------------------------------------" << std::endl;
}

int main()
{
    kinematics::kinematic_chain_factory chain_factory;
    auto chain = chain_factory.create();
    auto fk_solver = kinematics::forward_kinematics_solver();
    auto col_checker = kinematics::collision_checker();

    point p = fk_solver.compute_end_effector_position(chain);
    col_checker.check_collisions(chain);

    std::cout << "End effector position (x, y, z): " << p.x << ", " << p.y << ", " << p.z << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    chain->set_state(M_PI / 4, -M_PI / 2, -M_PI / 2, -M_PI / 2, 0.06);

    p = fk_solver.compute_end_effector_position(chain);
    col_checker.check_collisions(chain);

    std::cout << "End effector position (x, y, z): " << p.x << ", " << p.y << ", " << p.z << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    chain->set_state(-M_PI / 4, M_PI / 2, -3 * M_PI / 4, M_PI / 4, 0);

    p = fk_solver.compute_end_effector_position(chain);
    col_checker.check_collisions(chain);

    std::cout << "End effector position (x, y, z): " << p.x << ", " << p.y << ", " << p.z << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    chain->set_state(M_PI / 2, M_PI / 4, -M_PI / 6, 0, 0.05);

    p = fk_solver.compute_end_effector_position(chain);
    col_checker.check_collisions(chain);

    std::cout << "End effector position (x, y, z): " << p.x << ", " << p.y << ", " << p.z << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    chain->set_state(-M_PI / 4, 40 * M_PI / 180, 140 * M_PI / 180, 0, 0.05);

    p = fk_solver.compute_end_effector_position(chain);
    col_checker.check_collisions(chain);

    std::cout << "End effector position (x, y, z): " << p.x << ", " << p.y << ", " << p.z << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    const auto time_finder = planner::distance_travel_time_finder();
    auto planner = planner::trajectory_planner(time_finder, col_checker, chain);
    const double frequency = 150;

    std::vector<kinematics::dynamic_joint_state> start = {
        kinematics::dynamic_joint_state(0, -M_PI / 2, M_PI / 2, M_PI / 6, 2 * M_PI / 3),
        kinematics::dynamic_joint_state(0, -M_PI / 2, M_PI / 2, M_PI / 6, 2 * M_PI / 3),
        kinematics::dynamic_joint_state(0, -5 * M_PI / 6, 5 * M_PI / 6, M_PI / 4, M_PI),
        kinematics::dynamic_joint_state(0, -5 * M_PI / 6, 5 * M_PI / 6, M_PI / 4, M_PI),
        kinematics::dynamic_joint_state(0, 0, 0.05, 0.01, 0.05)};

    std::vector<kinematics::dynamic_joint_state> end = {
        kinematics::dynamic_joint_state(M_PI / 4, -M_PI / 2, M_PI / 2, M_PI / 6, 2 * M_PI / 3),
        kinematics::dynamic_joint_state(-M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 6, 2 * M_PI / 3),
        kinematics::dynamic_joint_state(-M_PI / 2, -5 * M_PI / 6, 5 * M_PI / 6, M_PI / 4, M_PI),
        kinematics::dynamic_joint_state(-M_PI / 2, -5 * M_PI / 6, 5 * M_PI / 6, M_PI / 4, M_PI),
        kinematics::dynamic_joint_state(0.05, 0, 0.05, 0.01, 0.05)};

    print_trajectory(planner.plan(start, end, frequency));

    start[0].set_value(M_PI / 4);
    end[4].set_value(0.05);
    start[1].set_value(-M_PI / 2);
    start[2].set_value(-M_PI / 2);
    start[3].set_value(-M_PI / 2);
    start[4].set_value(0.01);

    print_trajectory(planner.plan(start, end, frequency));

    start[0].set_value(0);
    start[1].set_value(0);
    start[2].set_value(0);
    start[3].set_value(0);
    start[4].set_value(0);
    end[0].set_value(-M_PI/4);
    end[1].set_value(M_PI/2);
    end[2].set_value(-3*M_PI/4);
    end[3].set_value(M_PI/4);
    end[4].set_value(0);
    print_trajectory(planner.plan(start, end, frequency));

    start[0].set_value(M_PI/4);
    start[1].set_value(-M_PI/18);
    start[2].set_value(2*M_PI/9);
    start[3].set_value(0);
    start[4].set_value(0);
    end[0].set_value(M_PI/4);
    end[1].set_value(-M_PI/18);
    end[2].set_value(5*M_PI/6);
    end[3].set_value(0);
    end[4].set_value(0);
    print_trajectory(planner.plan(start, end, frequency));

    start[0].set_value(-M_PI/2);
    start[1].set_value(-M_PI/2);
    start[2].set_value(5*M_PI/6);
    start[3].set_value(M_PI/6);
    start[4].set_value(0);
    end[0].set_value(-M_PI/2);
    end[1].set_value(0);
    end[2].set_value(5*M_PI/6);
    end[3].set_value(M_PI/6);
    end[4].set_value(0.05);
    print_trajectory(planner.plan(start, end, frequency));

    return 0;
}
