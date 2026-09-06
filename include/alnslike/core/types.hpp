#pragma once
#include <chrono>
#include <cstddef>
#include <random>

namespace alnslike::core {

using Cost = double;
using Rng = std::mt19937_64;  // Возможно, стоит заменить

struct SolutionDelta {
    Cost objective_delta{0.0};
    Cost feasibility_delta{0.0};

    constexpr bool IsImproving() const noexcept {
        if (feasibility_delta < 0.0) {
            return true;
        }
        if (feasibility_delta > 0.0) {
            return false;
        }
        return objective_delta < 0.0;
    }

    constexpr bool IsFeasibleTransition() const noexcept { return feasibility_delta <= 0.0; }
};

struct SolverState {
    std::size_t current_iteration{0};
    std::size_t iterations_without_improvement{0};
    std::chrono::duration<double> elapsed_time{0.0};
    Cost current_best_cost{0.0};
};

struct OperatorReward {
    Cost old_cost{0.0};
    Cost new_cost{0.0};
    Cost feasibility_violation_delta{0.0};
    bool is_accepted{false};
    bool is_new_best{false};
};

struct StepResult {
    SolutionDelta total_delta{};
    bool is_accepted{false};
    bool is_new_best{false};

    std::size_t selected_primary_index{0};
    std::size_t selected_secondary_index{0};
};

}  // namespace alnslike::core