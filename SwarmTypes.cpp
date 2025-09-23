#include "include/SwarmTypes.h"
#include <cmath>

namespace SwarmControl {

// Position3D implementations
    double Position3D::distance(const Position3D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    double Position3D::magnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }

// Velocity3D implementations
    double Velocity3D::magnitude() const {
        return std::sqrt(vx*vx + vy*vy + vz*vz);
    }

} // namespace SwarmControl//
// Created by yv on 22.09.2025.
//
