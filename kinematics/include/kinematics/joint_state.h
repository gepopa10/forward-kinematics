#pragma once

#include <iostream>
#include <algorithm>

namespace kinematics
{
    struct joint_state
    {
        joint_state() = delete;
        joint_state(double value, double min, double max) : _min(min), _max(max)
        {
            _value = std::max(_min, std::min(value, _max));
        }
        double get_value() const { return _value; };
        double get_min() const { return _min; };
        double get_max() const { return _max; };
        void set_value(double value)
        {
            if (value > _max || value < _min)
            {
                std::cout << "Unreachable joint value: " << value << " min: " << _min << " max: " << _max << std::endl;
            }
            _value = std::max(_min, std::min(value, _max));
        }

    private:
        double _value;
        double _min;
        double _max;
    };
} // namespace kinematics
