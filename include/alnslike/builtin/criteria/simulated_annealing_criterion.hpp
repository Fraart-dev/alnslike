#pragma once
#include <alnslike/core/acceptance_criterion.hpp>
#include <cmath>
#include <optional>
#include <random>
#include <stdexcept>

namespace alnslike::builtin::criteria {

class SimulatedAnnealingCriterion final : public core::AcceptanceCriterion {
public:
    static constexpr double kFreezeTemperature = 1e-9;

    static constexpr double kDefaultInitialTemperature = 1000.0;
    static constexpr double kDefaultFinalTemperature = 0.0001;
    static constexpr double kDefaultCoolingRate = 0.99975;

    struct Configuration {
        double initial_temperature{kDefaultInitialTemperature};

        // Используется, только если задан max_iterations
        double final_temperature{kDefaultFinalTemperature};

        // Взаимоисключающие опции:
        //     Если задан max_iterations, коэффициент рассчитывается
        //     автоматически. Иначе используется значение cooling_rate.
        std::optional<std::size_t> max_iterations{std::nullopt};
        std::optional<double> cooling_rate{std::nullopt};
    };

    SimulatedAnnealingCriterion() : SimulatedAnnealingCriterion(Configuration{}) {}

    explicit SimulatedAnnealingCriterion(const Configuration& config)
        : current_temperature_(config.initial_temperature), cooling_rate_(CalculateCoolingRate(config)) {}

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
    }

private:
    double current_temperature_;
    double cooling_rate_;
    std::uniform_real_distribution<double> random_double_{0.0, 1.0};

    static void ValidateConfiguration(const Configuration& config) {
        if (config.initial_temperature <= 0.0) {
            throw std::invalid_argument(
                "SimulatedAnnealingCriterion: начальная температура должна "
                "быть больше нуля.");
        }

        if (config.max_iterations.has_value() && config.cooling_rate.has_value()) {
            throw std::invalid_argument(
                "SimulatedAnnealingCriterion: нельзя одновременно задавать "
                "'max_iterations' и 'cooling_rate'.");
        }

        if (config.max_iterations.has_value()) {
            if (config.max_iterations.value() == 0) {
                throw std::invalid_argument(
                    "SimulatedAnnealingCriterion: количество итераций "
                    "(max_iterations) должно быть больше нуля.");
            }
            if (config.final_temperature <= 0.0 || config.final_temperature >= config.initial_temperature) {
                throw std::invalid_argument(
                    "SimulatedAnnealingCriterion: конечная температура должна "
                    "быть в диапазоне (0.0, initial_temperature).");
            }
        }
    }

    static double CalculateCoolingRate(const Configuration& config) {
        ValidateConfiguration(config);

        double rate = kDefaultCoolingRate;

        if (config.max_iterations.has_value()) {
            const double ratio = config.final_temperature / config.initial_temperature;
            rate = std::exp(std::log(ratio) / static_cast<double>(config.max_iterations.value()));
        } else if (config.cooling_rate.has_value()) {
            rate = config.cooling_rate.value();
        }

        if (rate <= 0.0 || rate >= 1.0 || std::isnan(rate)) {
            throw std::invalid_argument(
                "SimulatedAnnealingCriterion: рассчитанный или заданный "
                "коэффициент охлаждения вне диапазона (0.0, 1.0).");
        }

        return rate;
    }
};

}  // namespace alnslike::builtin::criteria