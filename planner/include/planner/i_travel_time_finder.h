#pragma once

namespace planner
{
    class i_travel_time_finder
    {
    public:
        virtual ~i_travel_time_finder() = default;
        virtual double compute_time_of_travel_secs(double start, double end, double max_vel, double max_acc) const = 0;
    };
} // namespace planner
