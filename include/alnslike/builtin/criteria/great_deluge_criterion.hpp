#pragma once
#include <alnslike/core/acceptance_criterion.hpp>
#include <stdexcept>

namespace alnslike::builtin::criteria {

class GreatDelugeCriterion final : public core::AcceptanceCriterion {
public:
    static constexpr double kDefaultInitialWaterLevel = 1000.0;
    static constexpr double kDefaultRainSpeed = 1.0;

    struct Configuration {
        double initial_water_level{kDefaultInitialWaterLevel};
        double rain_speed{kDefaultRainSpeed};
    };

    GreatDelugeCriterion() : GreatDelugeCriterion(Configuration{}) {}

    explicit GreatDelugeCriterion(const Configuration& config)
        : current_water_level_(config.initial_water_level), rain_speed_(config.rain_speed) {
        ValidateConfiguration(config);
    }

    [[nodiscard]] bool Accept(core::SolutionDelta /*delta*/, core::Cost current_cost, core::Cost new_cost,
                              const core::SolverState& /*state*/, core::Rng& /*rng*/
                              ) noexcept override {
        return (new_cost <= current_cost) || (new_cost <= current_water_level_);
    }

    void Update(core::SolutionDelta /*delta*/, bool /*is_accepted*/, const core::SolverState& /*state*/
                ) noexcept override {
        current_water_level_ -= rain_speed_;
    }

private:
    double current_water_level_;
    double rain_speed_;

    static void ValidateConfiguration(const Configuration& config) {
        if (config.rain_speed <= 0.0) {
            throw std::invalid_argument(
                "GreatDelugeCriterion: скорость снижения барьера должна быть "
                "больше нуля.");
        }
    }
};

}  // namespace alnslike::builtin::criteria