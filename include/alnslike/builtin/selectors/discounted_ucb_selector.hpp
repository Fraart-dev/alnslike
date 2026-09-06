#pragma once
#include <algorithm>
#include <alnslike/builtin/selectors/reward_weights.hpp>
#include <alnslike/core/operator_selector.hpp>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace alnslike::builtin::selectors {

class DiscountedUcbSelector final : public core::OperatorSelector {
public:
    static constexpr double kDefaultExplorationConstant = std::numbers::sqrt2;
    static constexpr double kDefaultGamma = 0.9985;

    struct Configuration {
        double exploration_constant{kDefaultExplorationConstant};
        double gamma{kDefaultGamma};
        RewardWeights weights{};
    };

    explicit DiscountedUcbSelector(std::size_t number_of_operators)
        : DiscountedUcbSelector(number_of_operators, Configuration{}) {}

    explicit DiscountedUcbSelector(std::size_t number_of_operators, const Configuration& config)
        : exploration_constant_(config.exploration_constant),
          gamma_(config.gamma),
          weights_(config.weights),
          statistics_(ValidateOperatorsCount(number_of_operators)) {
        ValidateConfiguration(config);
        inverse_sqrt_gamma_ = 1.0 / std::sqrt(gamma_);
    }

    [[nodiscard]] std::size_t Select(core::Rng& /*rng*/) override {
        const double ln_total_selections = std::log(std::max(1.0, total_selections_));
        const double exploration_numerator = exploration_constant_ * std::sqrt(ln_total_selections);

        std::size_t best_operator_index = 0;
        double highest_ucb_value = std::numeric_limits<double>::lowest();

        for (std::size_t index = 0; index < statistics_.size(); ++index) {
            const auto& record = statistics_[index];

            const double ucb_value =
                (record.average_reward + exploration_numerator) * record.inverse_sqrt_times_selected;

            if (ucb_value > highest_ucb_value) {
                highest_ucb_value = ucb_value;
                best_operator_index = index;
            }
        }

        return best_operator_index;
    }

    void UpdateWeights(std::size_t operator_index, const core::OperatorReward& reward) noexcept override {
        const double weighted_reward = weights_.Determine(reward);

        total_selections_ = gamma_ * (total_selections_ + 1.0);

        for (auto& record : statistics_) {
            record.total_reward *= gamma_;
            record.times_selected *= gamma_;
            record.inverse_sqrt_times_selected *= inverse_sqrt_gamma_;
        }

        auto& selected_record = statistics_[operator_index];
        selected_record.total_reward += weighted_reward;
        selected_record.times_selected += 1.0;

        selected_record.average_reward = selected_record.total_reward / selected_record.times_selected;
        selected_record.inverse_sqrt_times_selected = 1.0 / std::sqrt(selected_record.times_selected);
    }

    void ResetPeriod() noexcept override {
        for (auto& record : statistics_) {
            record.total_reward = 0.0;
            record.times_selected = 0.0;
            record.average_reward = std::numeric_limits<double>::max();
            record.inverse_sqrt_times_selected = 0.0;
        }
        total_selections_ = 0.0;
    }

private:
    struct OperatorRecord {
        double total_reward{0.0};
        double times_selected{0.0};

        // Оптимистичны изначально, чтобы исследовать все операторы
        double average_reward{std::numeric_limits<double>::max()};
        double inverse_sqrt_times_selected{0.0};
    };

    double exploration_constant_;
    double gamma_;
    RewardWeights weights_;

    std::vector<OperatorRecord> statistics_;
    double total_selections_{0.0};
    double inverse_sqrt_gamma_{1.0};

    static std::size_t ValidateOperatorsCount(std::size_t number_of_operators) {
        if (number_of_operators == 0) {
            throw std::invalid_argument(
                "DiscountedUcbSelector: количество операторов должно быть "
                "больше нуля.");
        }
        return number_of_operators;
    }

    static void ValidateConfiguration(const Configuration& config) {
        if (config.gamma <= 0.0 || config.gamma > 1.0) {
            throw std::invalid_argument(
                "DiscountedUcbSelector: параметр gamma должен быть в диапазоне "
                "(0, 1].");
        }
    }
};

}  // namespace alnslike::builtin::selectors