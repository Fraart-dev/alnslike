#pragma once
#include <algorithm>
#include <alnslike/builtin/selectors/reward_weights.hpp>
#include <alnslike/core/operator_selector.hpp>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

namespace alnslike::builtin::selectors {

class Exp3Selector final : public core::OperatorSelector {
public:
    static constexpr double kDefaultGamma = 0.1;

    struct Configuration {
        double gamma{kDefaultGamma};
        RewardWeights weights{};
    };

    explicit Exp3Selector(std::size_t number_of_operators) : Exp3Selector(number_of_operators, Configuration{}) {}

    explicit Exp3Selector(std::size_t number_of_operators, const Configuration& config)
        : gamma_(config.gamma),
          reward_weights_(config.weights),
          weights_(ValidateOperatorsCount(number_of_operators), 1.0),
          probabilities_(number_of_operators, 1.0 / static_cast<double>(number_of_operators)),
          random_double_(0.0, 1.0) {
        ValidateConfiguration(config);
    }

    [[nodiscard]] std::size_t Select(core::Rng& rng) override {
        const double threshold = random_double_(rng);
        double cumulative = 0.0;

        for (std::size_t i = 0; i < probabilities_.size() - 1; ++i) {
            cumulative += probabilities_[i];
            if (threshold <= cumulative) {
                return i;
            }
        }
        return probabilities_.size() - 1;
    }

    void UpdateWeights(std::size_t operator_index, const core::OperatorReward& reward) noexcept override {
        const double weighted_reward = reward_weights_.Determine(reward);
        const double probability = probabilities_[operator_index];
        const double exponent = (gamma_ * weighted_reward) / (probability * static_cast<double>(weights_.size()));

        constexpr double kMaxExponentThreshold = 10.0;
        weights_[operator_index] *= std::exp(std::min(exponent, kMaxExponentThreshold));

        RecalculateProbabilities();
    }

    void ResetPeriod() noexcept override {
        std::fill(weights_.begin(), weights_.end(), 1.0);
        RecalculateProbabilities();
    }

private:
    void RecalculateProbabilities() noexcept {
        double total_weight = 0.0;
        for (double weight : weights_) {
            total_weight += weight;
        }

        const double exploration_part = gamma_ / static_cast<double>(weights_.size());
        const double exploitation_factor = (1.0 - gamma_) / total_weight;

        for (std::size_t i = 0; i < weights_.size(); ++i) {
            probabilities_[i] = (exploitation_factor * weights_[i]) + exploration_part;
        }
    }

    double gamma_;
    RewardWeights reward_weights_;

    std::vector<double> weights_;
    std::vector<double> probabilities_;
    std::uniform_real_distribution<double> random_double_;

    static std::size_t ValidateOperatorsCount(std::size_t number_of_operators) {
        if (number_of_operators == 0) {
            throw std::invalid_argument("Exp3Selector: количество операторов должно быть больше нуля.");
        }
        return number_of_operators;
    }

    static void ValidateConfiguration(const Configuration& config) {
        if (config.gamma <= 0.0 || config.gamma > 1.0) {
            throw std::invalid_argument(
                "Exp3Selector: параметр gamma должен быть в диапазоне (0.0, "
                "1.0].");
        }
    }
};

}  // namespace alnslike::builtin::selectors