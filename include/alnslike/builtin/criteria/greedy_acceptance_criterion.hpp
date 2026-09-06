#pragma once
#include <alnslike/core/acceptance_criterion.hpp>

namespace alnslike::builtin::criteria {

class GreedyAcceptanceCriterion final : public core::AcceptanceCriterion {
public:
    GreedyAcceptanceCriterion() = default;

    bool Accept(core::SolutionDelta /*delta*/, core::Cost current_cost, core::Cost new_cost,
                const core::SolverState& /*state*/, core::Rng& /*rng*/
                ) noexcept override {
        return new_cost <= current_cost;
    }

    void Update(core::SolutionDelta /*delta*/, bool /*is_accepted*/, const core::SolverState& /*state*/
                ) noexcept override {}
};

}  // namespace alnslike::builtin::criteria