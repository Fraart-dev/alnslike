#pragma once
#include <alnslike/core/types.hpp>

namespace alnslike::core {

class AcceptanceCriterion {
public:
    virtual ~AcceptanceCriterion() = default;

    virtual bool Accept(SolutionDelta delta, Cost current_cost, Cost new_cost, const SolverState& state,
                        Rng& rng) noexcept = 0;

    virtual void Update(SolutionDelta delta, bool is_accepted, const SolverState& state) noexcept = 0;
};

}  // namespace alnslike::core