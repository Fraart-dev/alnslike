#pragma once
#include <alnslike/builtin/selectors/reward_weights.hpp>
#include <alnslike/core/operator_selector.hpp>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace alnslike::builtin::selectors {

class EpsilonGreedySelector final : public core::OperatorSelector {
public:
    static constexpr double kDefaultEpsilon = 0.1;

    struct Configuration {
        double epsilon{kDefaultEpsilon};
        RewardWeights weights{};
    };

    explicit EpsilonGreedySelector(std::size_t number_of_operators)
        : EpsilonGreedySelector(number_of_operators, Configuration{}) {}

    explicit EpsilonGreedySelector(std::size_t number_of_operators, const Configuration& config)
        : epsilon_(config.epsilon),
          weights_(config.weights),
          statistics_(ValidateOperatorsCount(number_of_operators)),
          random_double_(0.0, 1.0),
          random_index_(0, number_of_operators - 1) {
        ValidateConfiguration(config);
    }

    [[nodiscard]] std::size_t Select(core::Rng& rng) override {
        if (random_double_(rng) < epsilon_) {
            return random_index_(rng);
        }

        std::size_t best_operator_index = 0;
        double highest_average_reward = std::numeric_limits<double>::lowest();

        for (std::size_t index = 0; index < statistics_.size(); ++index) {
            const double average_reward = MeanReward(statistics_[index]);
            if (average_reward > highest_average_reward) {
                highest_average_reward = average_reward;
                best_operator_index = index;
            }
        }

        return best_operator_index;
    }

    void UpdateWeights(std::size_t operator_index, const core::OperatorReward& reward) noexcept override {
        const double weighted_reward = weights_.Determine(reward);
        statistics_[operator_index].total_reward += weighted_reward;
        statistics_[operator_index].times_selected += 1.0;
    }

    void ResetPeriod() noexcept override {
        for (auto& record : statistics_) {
            record.total_reward = 0.0;
            record.times_selected = 0.0;
        }
    }

private:
    struct OperatorRecord {
        double total_reward{0.0};
        double times_selected{0.0};
    };

    double epsilon_;
    RewardWeights weights_;

    std::vector<OperatorRecord> statistics_;
    std::uniform_real_distribution<double> random_double_;
    std::uniform_int_distribution<std::size_t> random_index_;

    static std::size_t ValidateOperatorsCount(std::size_t count) {
        if (count == 0) {
            throw std::invalid_argument(
                "EpsilonGreedySelector: количество операторов должно быть "
                "больше нуля.");
        }
        return count;
    }

    static void ValidateConfiguration(const Configuration& config) {
        if (config.epsilon < 0.0 || config.epsilon > 1.0) {
            throw std::invalid_argument(
                "EpsilonGreedySelector: параметр epsilon должен быть в "
                "диапазоне [0.0, 1.0].");
        }
    }

    static double MeanReward(const OperatorRecord& record) noexcept {
        if (record.times_selected == 0.0) {
            // Приоритет неиспользованным операторам
            return std::numeric_limits<double>::max();
        }
        return record.total_reward / record.times_selected;
    }
};

}  // namespace alnslike::builtin::selectors