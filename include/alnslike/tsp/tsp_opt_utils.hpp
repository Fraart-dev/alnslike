#pragma once

#include <alnslike/core/types.hpp>
#include <alnslike/tsp/tsp_solution.hpp>
#include <algorithm>
#include <utility>

namespace metaheuristic::tsp {

inline core::Cost EvaluateTwoOpt(
    const TspSolution& solution,
    std::size_t first_index,
    std::size_t second_index
) noexcept {
    if (first_index > second_index) {
        std::swap(first_index, second_index);
    }

    const auto& route = solution.GetRoute();
    const std::size_t route_size = route.size();

    if (first_index == second_index ||
        first_index + 1 == second_index ||
        (first_index == 0 && second_index == route_size - 1)) {
        return 0.0;
    }

    const auto* instance = solution.GetInstance();
    const NodeId first_node = route[first_index];
    const NodeId first_next_node = route[first_index + 1];
    const NodeId second_node = route[second_index];
    const NodeId second_next_node = route[second_index + 1 == route_size ? 0 : second_index + 1];

    const core::Cost removed_edges = instance->Distance(first_node, first_next_node) +
                                     instance->Distance(second_node, second_next_node);
    const core::Cost added_edges = instance->Distance(first_node, second_node) +
                                   instance->Distance(first_next_node, second_next_node);
    return added_edges - removed_edges;
}

inline void ApplyTwoOpt(
    TspSolution::Mutator& mutator,
    std::size_t first_index,
    std::size_t second_index,
    core::Cost delta
) noexcept {
    if (first_index > second_index) {
        std::swap(first_index, second_index);
    }
    if (first_index == second_index || first_index + 1 == second_index) {
        return;
    }

    auto& route = mutator.Route();
    auto& positions = mutator.Positions();

    std::reverse(route.begin() + static_cast<std::ptrdiff_t>(first_index + 1),
                  route.begin() + static_cast<std::ptrdiff_t>(second_index + 1));

    for (std::size_t route_index = first_index + 1; route_index <= second_index; ++route_index) {
        positions[route[route_index]] = route_index;
    }

    mutator.Cost() += delta;
}

//   A - узел first_index
//   B - узел first_index + 1
//   C - узел second_index
//   D - узел second_index + 1
//   E - узел third_index
//   F - узел third_index + 1
enum class ThreeOptReconnectionCase : uint8_t {
    None = 0,          
    ReverseBC = 1,    
    ReverseDE = 2,    
    ReverseBE = 3,    
    SwapBCDE = 4,     
    SwapReversedBCDE = 5,
    SwapBCReversedDE = 6,
    ReverseBothSegments = 7
};

struct ThreeOptMove {
    core::Cost delta{0.0};
    ThreeOptReconnectionCase reconnection_case{ThreeOptReconnectionCase::None};
};


// Реализация некрасивая, но так заметно быстрее
template <bool ExcludeTwoOptMoves>
[[nodiscard]] inline ThreeOptMove EvaluateThreeOpt(
    const TspSolution& solution,
    std::size_t first_index,
    std::size_t second_index,
    std::size_t third_index
) noexcept {
    const auto& route = solution.GetRoute();
    const TspInstance* instance = solution.GetInstance();
    const std::size_t route_size = route.size();

    if (first_index >= second_index || second_index >= third_index ||
        first_index + 1 == second_index || second_index + 1 == third_index ||
        (first_index == 0 && third_index == route_size - 1)) {
        return {.delta = 0.0, .reconnection_case = ThreeOptReconnectionCase::None};
    }

    const NodeId node_a = route[first_index];
    const NodeId node_b = route[first_index + 1];
    const NodeId node_c = route[second_index];
    const NodeId node_d = route[second_index + 1];
    const NodeId node_e = route[third_index];
    const NodeId node_f = route[third_index + 1 == route_size ? 0 : third_index + 1];

    const core::Cost distance_a_b = instance->Distance(node_a, node_b);
    const core::Cost distance_c_d = instance->Distance(node_c, node_d);
    const core::Cost distance_e_f = instance->Distance(node_e, node_f);

    const core::Cost distance_a_c = instance->Distance(node_a, node_c);
    const core::Cost distance_b_d = instance->Distance(node_b, node_d);
    const core::Cost distance_c_e = instance->Distance(node_c, node_e);
    const core::Cost distance_d_f = instance->Distance(node_d, node_f);
    const core::Cost distance_a_e = instance->Distance(node_a, node_e);
    const core::Cost distance_b_f = instance->Distance(node_b, node_f);
    const core::Cost distance_a_d = instance->Distance(node_a, node_d);
    const core::Cost distance_b_e = instance->Distance(node_b, node_e);
    const core::Cost distance_c_f = instance->Distance(node_c, node_f);

    const core::Cost removed_edges = distance_a_b + distance_c_d + distance_e_f;

    core::Cost best_delta = 0.0;
    ThreeOptReconnectionCase best_case = ThreeOptReconnectionCase::None;

    core::Cost current_delta = 0.0;

    if constexpr (!ExcludeTwoOptMoves) {
        current_delta = (distance_a_c + distance_b_d + distance_e_f) - removed_edges;
        if (current_delta < best_delta) { best_delta = current_delta; best_case = ThreeOptReconnectionCase::ReverseBC; }

        current_delta = (distance_a_b + distance_c_e + distance_d_f) - removed_edges;
        if (current_delta < best_delta) { best_delta = current_delta; best_case = ThreeOptReconnectionCase::ReverseDE; }

        current_delta = (distance_a_e + distance_b_f + distance_c_d) - removed_edges;
        if (current_delta < best_delta) { best_delta = current_delta; best_case = ThreeOptReconnectionCase::ReverseBE; }
    }

    current_delta = (distance_a_d + distance_b_e + distance_c_f) - removed_edges;
    if (current_delta < best_delta) { best_delta = current_delta; best_case = ThreeOptReconnectionCase::SwapBCDE; }

    current_delta = (distance_a_d + distance_c_e + distance_b_f) - removed_edges;
    if (current_delta < best_delta) { best_delta = current_delta; best_case = ThreeOptReconnectionCase::SwapReversedBCDE; }

    current_delta = (distance_a_e + distance_b_d + distance_c_f) - removed_edges;
    if (current_delta < best_delta) { best_delta = current_delta; best_case = ThreeOptReconnectionCase::SwapBCReversedDE; }

    current_delta = (distance_a_c + distance_b_e + distance_d_f) - removed_edges;
    if (current_delta < best_delta) { best_delta = current_delta; best_case = ThreeOptReconnectionCase::ReverseBothSegments; }

    return {.delta = best_delta, .reconnection_case = best_case};
}


inline void ApplyThreeOpt(
    TspSolution::Mutator& mutator,
    std::size_t first_index,
    std::size_t second_index,
    std::size_t third_index,
    const ThreeOptMove& move
) noexcept {
    if (move.reconnection_case == ThreeOptReconnectionCase::None) {
        return;
    }

    auto& route = mutator.Route();
    auto& positions = mutator.Positions();

    auto it_first = route.begin() + static_cast<std::ptrdiff_t>(first_index + 1);
    auto it_second = route.begin() + static_cast<std::ptrdiff_t>(second_index + 1);
    auto it_third = route.begin() + static_cast<std::ptrdiff_t>(third_index + 1);

    switch (move.reconnection_case) {
        case ThreeOptReconnectionCase::ReverseBC:
            std::reverse(it_first, it_second);
            break;
        case ThreeOptReconnectionCase::ReverseDE:
            std::reverse(it_second, it_third);
            break;
        case ThreeOptReconnectionCase::ReverseBE:
            std::reverse(it_first, it_third);
            break;
        case ThreeOptReconnectionCase::SwapBCDE:
            std::rotate(it_first, it_second, it_third);
            break;
        case ThreeOptReconnectionCase::SwapReversedBCDE:
            std::reverse(it_first, it_second);
            std::rotate(it_first, it_second, it_third);
            break;
        case ThreeOptReconnectionCase::SwapBCReversedDE:
            std::reverse(it_second, it_third);
            std::rotate(it_first, it_second, it_third);
            break;
        case ThreeOptReconnectionCase::ReverseBothSegments:
            std::reverse(it_first, it_second);
            std::reverse(it_second, it_third);
            break;
        default:
            return;
    }

    for (std::size_t route_index = first_index + 1; route_index <= third_index; ++route_index) {
        positions[route[route_index]] = route_index;
    }

    mutator.Cost() += move.delta;
}

} // namespace metaheuristic::tsp