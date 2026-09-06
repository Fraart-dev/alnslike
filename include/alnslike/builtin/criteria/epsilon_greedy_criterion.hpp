#pragma once
#include <alnslike/core/acceptance_criterion.hpp>
#include <random>
#include <stdexcept>

namespace alnslike::builtin::criteria {

class EpsilonGreedyAcceptanceCriterion final : public core::AcceptanceCriterion {
public:
    static constexpr double kDefaultEpsilon = 0.1;

    struct Configuration {
        double epsilon{kDefaultEpsilon};
    };

    EpsilonGreedyAcceptanceCriterion() : EpsilonGreedyAcceptanceCriterion(Configuration{}) {}

    explicit EpsilonGreedyAcceptanceCriterion(const Configuration& config) : epsilon_(config.epsilon) {
        ValidateConfiguration(config);
    }

    [[nodiscard]] bool Accept(core::SolutionDelta /*delta*/, core::Cost current_cost, core::Cost new_cost,
                              const core::SolverState& /*state*/, core::Rng& rng) noexcept override {
        if (new_cost <= current_cost) {
            return true;
        }

        return random_double_(rng) < epsilon_;
    }

    void Update(core::SolutionDelta /*delta*/, bool /*is_accepted*/, const core::SolverState& /*state*/
                ) noexcept override {}

private:
    double epsilon_;
    std::uniform_real_distribution<double> random_double_{0.0, 1.0};

    static void ValidateConfiguration(const Configuration& config) {
        if (config.epsilon < 0.0 || config.epsilon > 1.0) {
            throw std::invalid_argument(
                "EpsilonGreedyAcceptanceCriterion: параметр epsilon должен "
                "быть в диапазоне [0.0, 1.0].");
        }
    }
};

}  // namespace alnslike::builtin::criteria