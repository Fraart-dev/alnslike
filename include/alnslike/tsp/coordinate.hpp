#pragma once
#include <cmath>
#include <cstdint>
#include <limits>

namespace metaheuristic::tsp {

using NodeId = std::uint32_t;
inline constexpr NodeId kInvalidNode = std::numeric_limits<NodeId>::max();

struct Coordinate {
    double x{0.0};
    double y{0.0};

    [[nodiscard]] double DistanceTo(const Coordinate& other) const noexcept {
        const double dx = x - other.x;
        const double dy = y - other.y;
        return std::round(std::sqrt((dx*dx) + (dy*dy)));
    }

    [[nodiscard]] double DistanceSquared(const Coordinate& other) const noexcept {
        const double dx = x - other.x;
        const double dy = y - other.y;
        return (dx * dx) + (dy * dy);
    }
};

} // namespace metaheuristic::tsp