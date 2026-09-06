#pragma once
#include <algorithm>
#include <alnslike/core/acceptance_criterion.hpp>
#include <limits>
#include <stdexcept>
#include <vector>

namespace alnslike::builtin::criteria {

class DiversifiedLateAcceptanceCriterion final : public core::AcceptanceCriterion {
public:
    static constexpr std::size_t kDefaultHistoryLength = 1000;

    struct Configuration {
        std::size_t history_length{kDefaultHistoryLength};
    };

    DiversifiedLateAcceptanceCriterion() : DiversifiedLateAcceptanceCriterion(Configuration{}) {}

    explicit DiversifiedLateAcceptanceCriterion(const Configuration& config) : history_(config.history_length) {
        ValidateConfiguration(config);
    }

    [[nodiscard]] bool Accept(core::SolutionDelta delta, core::Cost current_cost, core::Cost new_cost,
                              const core::SolverState& /*state*/, core::Rng& /*rng*/
                              ) noexcept override {
        if (delta.feasibility_delta > 0.0) {
            return false;
        }

        cached_current_cost_ = current_cost;
        cached_new_cost_ = new_cost;

        if (!is_initialized_) [[unlikely]] {
            return new_cost <= current_cost;
        }

        return (new_cost <= current_cost) || (new_cost < max_history_cost_);
    }

    void Update(core::SolutionDelta /*delta*/, bool is_accepted, const core::SolverState& /*state*/
                ) noexcept override {
        if (!is_initialized_) [[unlikely]] {
            std::fill(history_.begin(), history_.end(), cached_current_cost_);
            max_history_cost_ = cached_current_cost_;
            max_cost_occurrences_ = history_.size();
            is_initialized_ = true;
        }

        const core::Cost current_active_cost = is_accepted ? cached_new_cost_ : cached_current_cost_;
        const core::Cost old_history_cost = history_[current_index_];

        const bool is_diversification = current_active_cost > old_history_cost;
        const bool is_intensification = (current_active_cost < old_history_cost) && is_accepted;

        if (is_diversification || is_intensification) {
            history_[current_index_] = current_active_cost;

            if (current_active_cost > max_history_cost_) {
                max_history_cost_ = current_active_cost;
                max_cost_occurrences_ = 1;
            } else {
                if (old_history_cost == max_history_cost_) {
                    --max_cost_occurrences_;
                }

                if (current_active_cost == max_history_cost_) {
                    ++max_cost_occurrences_;
                }

                if (max_cost_occurrences_ == 0) [[unlikely]] {
                    RecalculateMaximum();
                }
            }
        }

        ++current_index_;
        if (current_index_ == history_.size()) [[unlikely]] {
            current_index_ = 0;
        }
    }

private:
    void RecalculateMaximum() noexcept {
        max_history_cost_ = std::numeric_limits<core::Cost>::lowest();
        max_cost_occurrences_ = 0;

        for (const core::Cost cost : history_) {
            if (cost > max_history_cost_) {
                max_history_cost_ = cost;
                max_cost_occurrences_ = 1;
            } else if (cost == max_history_cost_) {
                ++max_cost_occurrences_;
            }
        }
    }

    std::vector<core::Cost> history_;

    std::size_t current_index_{0};
    std::size_t max_cost_occurrences_{0};

    core::Cost max_history_cost_{0.0};
    core::Cost cached_current_cost_{0.0};
    core::Cost cached_new_cost_{0.0};

    bool is_initialized_{false};

    static void ValidateConfiguration(const Configuration& config) {
        if (config.history_length == 0) {
            throw std::invalid_argument(
                "DiversifiedLateAcceptanceCriterion: длина истории "
                "(history_length) должна быть больше нуля.");
        }
    }
};

}  // namespace alnslike::builtin::criteria