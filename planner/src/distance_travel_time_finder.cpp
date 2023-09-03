#include <cmath>
#include <iostream>

#include "planner/distance_travel_time_finder.h"

namespace planner
{
    double distance_travel_time_finder::compute_time_of_travel_secs(double start, double end, double max_vel, double max_acc) const
    {
        // starts and finishes at rest
        // assume instant constant acceleration
        const double distance = std::abs(end - start);
        const double time_to_max_speed = max_vel / max_acc;
        const double distance_to_max_speed = 0.5 * max_acc * time_to_max_speed * time_to_max_speed;

        if (distance <= distance_to_max_speed)
        {
            return 2*std::sqrt(distance / max_acc);
        }
        const auto total_time = 2 * time_to_max_speed + (distance - 2 * distance_to_max_speed) / max_vel;
        return total_time;
    }
} // namespace planner
