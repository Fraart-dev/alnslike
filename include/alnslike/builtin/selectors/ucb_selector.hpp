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

class UcbSelector final : public core::OperatorSelector {
public:
    static constexpr double kDefaultExplorationConstant = std::numbers::sqrt2;

    struct Configuration {
        double exploration_constant{kDefaultExplorationConstant};
        RewardWeights weights{};
    };

    explicit UcbSelector(std::size_t number_of_operators) : UcbSelector(number_of_operators, Configuration{}) {}

    explicit UcbSelector(std::size_t number_of_operators, const Configuration& config)
        : exploration_constant_(config.exploration_constant),
          reward_weights_(config.weights),
          statistics_(ValidateOperatorsCount(number_of_operators)) {
        ValidateConfiguration(config);
    }

    [[nodiscard]] std::size_t Select(core::Rng& /*rng*/) override {
        const double ln_total_selections = std::log(std::max(1.0, total_selections_));

        const double exploration_numerator = exploration_constant_ * std::sqrt(ln_total_selections);

        std::size_t best_operator_index = 0;
        double highest_ucb_value = std::numeric_limits<double>::lowest();

        for (std::size_t index = 0; index < statistics_.size(); ++index) {
            const auto& record = statistics_[index];

            const double ucb_value =
                record.average_reward + (exploration_numerator * record.inverse_sqrt_times_selected);

            if (ucb_value > highest_ucb_value) {
                highest_ucb_value = ucb_value;
                best_operator_index = index;
            }
        }

        return best_operator_index;
    }

    void UpdateWeights(std::size_t operator_index, const core::OperatorReward& reward) noexcept override {
        const double weighted_reward = reward_weights_.Determine(reward);
        auto& record = statistics_[operator_index];

        record.total_reward += weighted_reward;
        record.times_selected += 1.0;
        total_selections_ += 1.0;

        record.average_reward = record.total_reward / record.times_selected;
        record.inverse_sqrt_times_selected = 1.0 / std::sqrt(record.times_selected);
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

        double average_reward{std::numeric_limits<double>::max()};
        double inverse_sqrt_times_selected{0.0};
    };

    double exploration_constant_;
    RewardWeights reward_weights_;

    std::vector<OperatorRecord> statistics_;
    double total_selections_{0.0};

    static std::size_t ValidateOperatorsCount(std::size_t number_of_operators) {
        if (number_of_operators == 0) {
            throw std::invalid_argument("UcbSelector: количество операторов должно быть больше нуля.");
        }
        return number_of_operators;
    }

    static void ValidateConfiguration(const Configuration& config) {
        if (config.exploration_constant < 0.0) {
            throw std::invalid_argument(
                "UcbSelector: параметр exploration_constant не может быть "
                "отрицательным.");
        }
    }
};

}  // namespace alnslike::builtin::selectors