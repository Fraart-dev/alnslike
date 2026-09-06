#pragma once
#include <algorithm>
#include <alnslike/builtin/selectors/reward_weights.hpp>
#include <alnslike/core/operator_selector.hpp>
#include <random>
#include <stdexcept>
#include <vector>

namespace alnslike::builtin::selectors {

class ClassicRouletteSelector final : public core::OperatorSelector {
public:
    static constexpr double kDefaultReactionFactorRho = 0.01;
    static constexpr double kDefaultMinWeight = 0.01;

    struct Configuration {
        double reaction_factor_rho{kDefaultReactionFactorRho};
        double min_weight{kDefaultMinWeight};
        RewardWeights weights{};
    };

    explicit ClassicRouletteSelector(std::size_t number_of_operators)
        : ClassicRouletteSelector(number_of_operators, Configuration{}) {}

    explicit ClassicRouletteSelector(std::size_t number_of_operators, const Configuration& config)
        : reaction_factor_rho_(config.reaction_factor_rho),
          min_weight_(config.min_weight),
          reward_weights_(config.weights),
          weights_(ValidateOperatorsCount(number_of_operators), 1.0),
          cached_total_weight_(static_cast<double>(number_of_operators)) {
        ValidateConfiguration(config);
    }

    [[nodiscard]] std::size_t Select(core::Rng& rng) override {
        double random_threshold = random_double_(rng) * cached_total_weight_;

        for (std::size_t i = 0; i < weights_.size() - 1; ++i) {
            random_threshold -= weights_[i];
            if (random_threshold <= 0.0) {
                return i;
            }
        }

        return weights_.size() - 1;
    }

    void UpdateWeights(std::size_t operator_index, const core::OperatorReward& reward) noexcept override {
        const double calculated_reward = reward_weights_.Determine(reward);
        const double old_weight = weights_[operator_index];

        const double new_weight = std::max(
            ((1.0 - reaction_factor_rho_) * old_weight) + (reaction_factor_rho_ * calculated_reward), min_weight_);

        weights_[operator_index] = new_weight;
        cached_total_weight_ += (new_weight - old_weight);
    }

    void ResetPeriod() noexcept override {
        std::fill(weights_.begin(), weights_.end(), 1.0);
        cached_total_weight_ = static_cast<double>(weights_.size());
    }

private:
    double reaction_factor_rho_;
    double min_weight_;
    RewardWeights reward_weights_;

    std::vector<double> weights_;
    double cached_total_weight_;
    std::uniform_real_distribution<double> random_double_{0.0, 1.0};

    static std::size_t ValidateOperatorsCount(std::size_t number_of_operators) {
        if (number_of_operators == 0) {
            throw std::invalid_argument(
                "ClassicRouletteSelector: количество операторов должно быть "
                "больше нуля.");
        }
        return number_of_operators;
    }

    static void ValidateConfiguration(const Configuration& config) {
        if (config.reaction_factor_rho < 0.0 || config.reaction_factor_rho > 1.0) {
            throw std::invalid_argument(
                "ClassicRouletteSelector: параметр reaction_factor_rho должен "
                "быть в диапазоне [0.0, 1.0].");
        }
        if (config.min_weight < 0.0) {
            throw std::invalid_argument(
                "ClassicRouletteSelector: параметр min_weight не может быть "
                "отрицательным.");
        }
    }
};

}  // namespace alnslike::builtin::selectors