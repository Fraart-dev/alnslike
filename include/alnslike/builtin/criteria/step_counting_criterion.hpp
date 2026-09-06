#pragma once
#include <alnslike/core/acceptance_criterion.hpp>
#include <stdexcept>

namespace alnslike::builtin::criteria {

class StepCountingCriterion final : public core::AcceptanceCriterion {
public:
    static constexpr std::size_t kDefaultStepPeriod = 50;

    struct Configuration {
        std::size_t step_period{kDefaultStepPeriod};
    };

    StepCountingCriterion() : StepCountingCriterion(Configuration{}) {}

    explicit StepCountingCriterion(const Configuration& config) : step_period_(config.step_period) {
        ValidateConfiguration(config);
    }

    [[nodiscard]] bool Accept(core::SolutionDelta /*delta*/, core::Cost current_cost, core::Cost new_cost,
                              const core::SolverState& /*state*/, core::Rng& /*rng*/
                              ) noexcept override {
        if (current_step_counter_ == 0) {
            current_threshold_ = current_cost;
        }

        return (new_cost <= current_cost) || (new_cost < current_threshold_);
    }

    void Update(core::SolutionDelta /*delta*/, bool /*is_accepted*/, const core::SolverState& /*state*/
                ) noexcept override {
        ++current_step_counter_;

        if (current_step_counter_ >= step_period_) {
            current_step_counter_ = 0;
        }
    }

private:
    std::size_t step_period_;
    std::size_t current_step_counter_{0};
    core::Cost current_threshold_{0.0};

    static void ValidateConfiguration(const Configuration& config) {
        if (config.step_period == 0) {
            throw std::invalid_argument(
                "StepCountingCriterion: период счетчика шагов (step_period) "
                "должен быть больше нуля.");
        }
    }
};

}  // namespace alnslike::builtin::criteria