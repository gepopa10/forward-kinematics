#pragma once

struct point {
    point(double x, double y, double z) : x(x), y(y), z(z) {}
    double x, y, z;

    bool operator==(const point& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};
