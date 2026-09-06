#pragma once
#include <alnslike/core/operator_selector.hpp>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace alnslike::builtin::selectors {

class ThompsonSamplingSelector final : public core::OperatorSelector {
public:
    explicit ThompsonSamplingSelector(std::size_t number_of_operators)
        : statistics_(ValidateOperatorsCount(number_of_operators)), gamma_alpha_(1.0, 1.0), gamma_beta_(1.0, 1.0) {}

    [[nodiscard]] std::size_t Select(core::Rng& rng) override {
        std::size_t best_operator_index = 0;
        double highest_sampled_theta = std::numeric_limits<double>::lowest();

        for (std::size_t i = 0; i < statistics_.size(); ++i) {
            gamma_alpha_.param(std::gamma_distribution<double>::param_type(statistics_[i].successes, 1.0));
            gamma_beta_.param(std::gamma_distribution<double>::param_type(statistics_[i].failures, 1.0));

            const double x = gamma_alpha_(rng);
            const double y = gamma_beta_(rng);
            const double theta = x / (x + y);

            if (theta > highest_sampled_theta) {
                highest_sampled_theta = theta;
                best_operator_index = i;
            }
        }
        return best_operator_index;
    }

    void UpdateWeights(std::size_t operator_index, const core::OperatorReward& reward) noexcept override {
        if (reward.is_accepted) {
            statistics_[operator_index].successes += 1.0;
        } else {
            statistics_[operator_index].failures += 1.0;
        }
    }

    void ResetPeriod() noexcept override {
        for (auto& record : statistics_) {
            record.successes = 1.0;
            record.failures = 1.0;
        }
    }

private:
    struct OperatorRecord {
        double successes{1.0};
        double failures{1.0};
    };

    std::vector<OperatorRecord> statistics_;
    std::gamma_distribution<double> gamma_alpha_;
    std::gamma_distribution<double> gamma_beta_;

    static std::size_t ValidateOperatorsCount(std::size_t number_of_operators) {
        if (number_of_operators == 0) {
            throw std::invalid_argument(
                "ThompsonSamplingSelector: количество операторов должно быть "
                "больше нуля.");
        }
        return number_of_operators;
    }
};

}  // namespace alnslike::builtin::selectors