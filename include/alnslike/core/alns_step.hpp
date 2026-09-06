#pragma once
#include <alnslike/core/operator.hpp>
#include <alnslike/core/operator_selector.hpp>
#include <alnslike/core/search_step.hpp>
#include <memory>
#include <stdexcept>
#include <vector>

namespace alnslike::core {

class AlnsStep : public SearchStep {
public:
    explicit AlnsStep(std::unique_ptr<OperatorSelector> ruin_selector,
                      std::unique_ptr<OperatorSelector> repair_selector,
                      std::vector<std::unique_ptr<RuinOperator>> ruin_operators,
                      std::vector<std::unique_ptr<RepairOperator>> repair_operators)
        : ruin_selector_(std::move(ruin_selector)),
          repair_selector_(std::move(repair_selector)),
          ruin_operators_(std::move(ruin_operators)),
          repair_operators_(std::move(repair_operators)) {
        if (!ruin_selector_ || !repair_selector_) {
            throw std::invalid_argument("AlnsStep: селектор не задан.");
        }
        if (ruin_operators_.empty() || repair_operators_.empty()) {
            throw std::invalid_argument("AlnsStep: отсутствуют операторы (ruin/repair).");
        }
    }

    StepResult Execute(Solution& current_solution, MoveCache& cache, AcceptanceCriterion& criterion,
                       const SolverState& state, Rng& rng) noexcept override {
        cache.Clear();

        const std::size_t ruin_index = ruin_selector_->Select(rng);
        const std::size_t repair_index = repair_selector_->Select(rng);

        auto& ruin_operator = ruin_operators_[ruin_index];
        auto& repair_operator = repair_operators_[repair_index];

        const Cost current_cost = current_solution.GetCost();

        const SolutionDelta ruin_delta = ruin_operator->Ruin(current_solution, cache, rng);
        const SolutionDelta repair_delta = repair_operator->Repair(current_solution, cache, rng);

        const SolutionDelta total_delta{
            .objective_delta = ruin_delta.objective_delta + repair_delta.objective_delta,
            .feasibility_delta = ruin_delta.feasibility_delta + repair_delta.feasibility_delta};

        const Cost new_cost = current_cost + total_delta.objective_delta;

        const bool is_accepted = criterion.Accept(total_delta, current_cost, new_cost, state, rng);
        criterion.Update(total_delta, is_accepted, state);

        StepResult result{.total_delta = total_delta,
                          .is_accepted = is_accepted,
                          .selected_primary_index = ruin_index,
                          .selected_secondary_index = repair_index};

        if (is_accepted && new_cost < state.current_best_cost && total_delta.IsImproving()) {
            result.is_new_best = true;
        }

        OperatorReward reward{.old_cost = current_cost,
                              .new_cost = new_cost,
                              .feasibility_violation_delta = total_delta.feasibility_delta,
                              .is_accepted = is_accepted,
                              .is_new_best = result.is_new_best};

        ruin_selector_->UpdateWeights(ruin_index, reward);
        repair_selector_->UpdateWeights(repair_index, reward);

        if (!is_accepted) {
            repair_operator->Undo(current_solution, cache);
            ruin_operator->Undo(current_solution, cache);
        }

        return result;
    }

private:
    std::unique_ptr<OperatorSelector> ruin_selector_;
    std::unique_ptr<OperatorSelector> repair_selector_;
    std::vector<std::unique_ptr<RuinOperator>> ruin_operators_;
    std::vector<std::unique_ptr<RepairOperator>> repair_operators_;
};

}  // namespace alnslike::core