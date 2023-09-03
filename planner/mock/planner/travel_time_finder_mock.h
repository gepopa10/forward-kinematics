#pragma once

#include <gmock/gmock.h>

#include "planner/i_travel_time_finder.h"
#include "kinematics/kinematic_chain.h"

namespace planner
{
    class travel_time_finder_mock : public i_travel_time_finder
    {
    public:
       MOCK_CONST_METHOD4(compute_time_of_travel_secs, double(double, double, double, double));
    };
} // namespace planner