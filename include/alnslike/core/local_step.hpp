#pragma once
#include <alnslike/core/operator.hpp>
#include <alnslike/core/operator_selector.hpp>
#include <alnslike/core/search_step.hpp>
#include <memory>
#include <stdexcept>
#include <vector>

namespace alnslike::core {

class LocalSearchStep final : public SearchStep {
public:
    explicit LocalSearchStep(std::unique_ptr<OperatorSelector> selector,
                             std::vector<std::unique_ptr<LocalOperator>> operators)
        : selector_(std::move(selector)), operators_(std::move(operators)) {
        if (!selector_) {
            throw std::invalid_argument("LocalSearchStep: селектор не задан.");
        }
        if (operators_.empty()) {
            throw std::invalid_argument("LocalSearchStep: список операторов пуст.");
        }
    }

    StepResult Execute(Solution& current_solution, MoveCache& /*cache*/, AcceptanceCriterion& criterion,
                       const SolverState& state, Rng& rng) noexcept override {
        const std::size_t selected_operator_index = selector_->Select(rng);
        auto& selected_operator = operators_[selected_operator_index];

        const Cost current_cost = current_solution.GetCost();
        const SolutionDelta delta = selected_operator->Propose(current_solution, rng);
        const Cost new_cost = current_cost + delta.objective_delta;

        const bool is_accepted = criterion.Accept(delta, current_cost, new_cost, state, rng);
        criterion.Update(delta, is_accepted, state);

        StepResult result{
            .total_delta = delta, .is_accepted = is_accepted, .selected_primary_index = selected_operator_index};

        if (is_accepted && new_cost < state.current_best_cost && delta.IsImproving()) {
            result.is_new_best = true;
        }

        OperatorReward reward{.old_cost = current_cost,
                              .new_cost = new_cost,
                              .feasibility_violation_delta = delta.feasibility_delta,
                              .is_accepted = is_accepted,
                              .is_new_best = result.is_new_best};

        selector_->UpdateWeights(selected_operator_index, reward);

        selected_operator->Finalize(current_solution, is_accepted);

        return result;
    }

private:
    std::unique_ptr<OperatorSelector> selector_;
    std::vector<std::unique_ptr<LocalOperator>> operators_;
};

}  // namespace alnslike::core