#pragma once
#include <algorithm>
#include <alnslike/core/acceptance_criterion.hpp>
#include <cmath>
#include <optional>
#include <random>
#include <stdexcept>

namespace alnslike::builtin::criteria {

class HeatingSimulatedAnnealingCriterion final : public core::AcceptanceCriterion {
public:
    static constexpr double kFreezeTemperature = 1e-9;

    static constexpr double kDefaultInitialTemperature = 1000.0;
    static constexpr double kDefaultMinimumTemperature = 0.0001;
    static constexpr double kDefaultCycleDecayFactor = 0.7;
    static constexpr double kDefaultCoolingRate = 0.99975;

    struct Configuration {
        double initial_temperature{kDefaultInitialTemperature};
        double cycle_decay_factor{kDefaultCycleDecayFactor};
        double minimum_temperature{kDefaultMinimumTemperature};

        // Взаимоисключающие опции:
        //     Если задан max_iterations, коэффициент рассчитывается
        //     автоматически. Иначе используется значение cooling_rate.
        std::optional<std::size_t> iterations_per_cycle{std::nullopt};
        std::optional<double> cooling_rate{std::nullopt};
    };

    HeatingSimulatedAnnealingCriterion() : HeatingSimulatedAnnealingCriterion(Configuration{}) {}

    explicit HeatingSimulatedAnnealingCriterion(const Configuration& config)
        : current_temperature_(config.initial_temperature),
          minimum_temperature_(config.minimum_temperature),
          cooling_rate_(CalculateCoolingRate(config)),
          cycle_decay_factor_(config.cycle_decay_factor),
          current_peak_temperature_(config.initial_temperature) {}

    [[nodiscard]] bool Accept(core::SolutionDelta /*delta*/, core::Cost current_cost, core::Cost new_cost,
                              const core::SolverState& /*state*/, core::Rng& rng) noexcept override {
        if (new_cost <= current_cost) {
            return true;
        }

        if (current_temperature_ < kFreezeTemperature) {
            return false;
        }

        const double objective_difference = current_cost - new_cost;
        const double acceptance_probability = std::exp(objective_difference / current_temperature_);

        return random_double_(rng) < acceptance_probability;
    }

    void Update(core::SolutionDelta /*delta*/, bool /*is_accepted*/, const core::SolverState& /*state*/
                ) noexcept override {
        current_temperature_ *= cooling_rate_;

        if (current_temperature_ <= minimum_temperature_) {
            current_peak_temperature_ *= cycle_decay_factor_;
            current_temperature_ = std::max(current_peak_temperature_, minimum_temperature_);
        }
    }

private:
    double current_temperature_;
    double minimum_temperature_;
    double cooling_rate_;
    double cycle_decay_factor_;
    double current_peak_temperature_;
    std::uniform_real_distribution<double> random_double_{0.0, 1.0};

    static void ValidateConfiguration(const Configuration& config) {
        if (config.initial_temperature <= 0.0) {
            throw std::invalid_argument(
                "HeatingSimulatedAnnealingCriterion: начальная температура "
                "должна быть больше нуля.");
        }
        if (config.minimum_temperature <= 0.0 || config.minimum_temperature >= config.initial_temperature) {
            throw std::invalid_argument(
                "HeatingSimulatedAnnealingCriterion: минимальная температура "
                "должна быть в диапазоне (0.0, initial_temperature).");
        }
        if (config.cycle_decay_factor <= 0.0 || config.cycle_decay_factor > 1.0) {
            throw std::invalid_argument(
                "HeatingSimulatedAnnealingCriterion: коэффициент затухания "
                "цикла должен быть в диапазоне (0.0, 1.0].");
        }

        if (config.iterations_per_cycle.has_value() && config.cooling_rate.has_value()) {
            throw std::invalid_argument(
                "HeatingSimulatedAnnealingCriterion: нельзя одновременно "
                "задавать 'iterations_per_cycle' и 'cooling_rate'.");
        }
        if (config.iterations_per_cycle.has_value() && config.iterations_per_cycle.value() == 0) {
            throw std::invalid_argument(
                "HeatingSimulatedAnnealingCriterion: количество итераций "
                "(iterations_per_cycle) должно быть больше нуля.");
        }
    }

    static double CalculateCoolingRate(const Configuration& config) {
        ValidateConfiguration(config);

        double rate = kDefaultCoolingRate;

        if (config.iterations_per_cycle.has_value()) {
            const double ratio = config.minimum_temperature / config.initial_temperature;
            rate = std::exp(std::log(ratio) / static_cast<double>(config.iterations_per_cycle.value()));
        } else if (config.cooling_rate.has_value()) {
            rate = config.cooling_rate.value();
        }

        if (rate <= 0.0 || rate >= 1.0 || std::isnan(rate)) {
            throw std::invalid_argument(
                "HeatingSimulatedAnnealingCriterion: рассчитанный или заданный "
                "коэффициент охлаждения вне диапазона (0.0, 1.0).");
        }

        return rate;
    }
};

}  // namespace alnslike::builtin::criteria