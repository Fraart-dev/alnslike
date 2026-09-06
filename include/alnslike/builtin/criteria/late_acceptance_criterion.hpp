#pragma once
#include <algorithm>
#include <alnslike/core/acceptance_criterion.hpp>
#include <stdexcept>
#include <vector>

namespace alnslike::builtin::criteria {

class LateAcceptanceCriterion final : public core::AcceptanceCriterion {
public:
    static constexpr std::size_t kDefaultHistoryLength = 1000;

    struct Configuration {
        std::size_t history_length{kDefaultHistoryLength};
    };

    LateAcceptanceCriterion() : LateAcceptanceCriterion(Configuration{}) {}

    explicit LateAcceptanceCriterion(const Configuration& config) : history_(config.history_length) {
        ValidateConfiguration(config);
    }

    [[nodiscard]] bool Accept(core::SolutionDelta /*delta*/, core::Cost current_cost, core::Cost new_cost,
                              const core::SolverState& /*state*/, core::Rng& /*rng*/
                              ) noexcept override {
        cached_current_cost_ = current_cost;
        cached_new_cost_ = new_cost;

        if (!is_initialized_) {
            return new_cost <= current_cost;
        }

        const core::Cost reference_past_cost = history_[current_index_];

        return (new_cost <= current_cost) || (new_cost <= reference_past_cost);
    }

    void Update(core::SolutionDelta /*delta*/, bool is_accepted, const core::SolverState& /*state*/
                ) noexcept override {
        if (!is_initialized_) {
            std::fill(history_.begin(), history_.end(), cached_current_cost_);
            is_initialized_ = true;
        }

        const core::Cost current_cost = is_accepted ? cached_new_cost_ : cached_current_cost_;

        history_[current_index_] = current_cost;
        current_index_ = (current_index_ + 1) % history_.size();
    }

private:
    std::vector<core::Cost> history_;
    std::size_t current_index_{0};
    bool is_initialized_{false};

    core::Cost cached_current_cost_{0.0};
    core::Cost cached_new_cost_{0.0};

    static void ValidateConfiguration(const Configuration& config) {
        if (config.history_length == 0) {
            throw std::invalid_argument(
                "LateAcceptanceCriterion: длина истории (history_length) "
                "должна быть больше нуля.");
        }
    }
};

}  // namespace alnslike::builtin::criteria