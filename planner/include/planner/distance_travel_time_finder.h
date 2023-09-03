#pragma once

#include "planner/i_travel_time_finder.h"

namespace planner
{
    class distance_travel_time_finder : public i_travel_time_finder
    {
    public:
        distance_travel_time_finder() = default;
        double compute_time_of_travel_secs(double start, double end, double max_vel, double max_acc) const override;
    };
} // namespace planner
