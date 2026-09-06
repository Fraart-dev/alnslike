#pragma once
#include <alnslike/core/acceptance_criterion.hpp>
#include <alnslike/core/solution.hpp>
#include <alnslike/core/types.hpp>

namespace alnslike::core {

class SearchStep {
public:
    virtual ~SearchStep() = default;

    virtual StepResult Execute(Solution& current_solution, MoveCache& cache, AcceptanceCriterion& criterion,
                               const SolverState& state, Rng& rng) noexcept = 0;
};

}  // namespace alnslike::core