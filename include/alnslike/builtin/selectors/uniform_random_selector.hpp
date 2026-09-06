#pragma once
#include <alnslike/core/operator_selector.hpp>
#include <random>
#include <stdexcept>

namespace alnslike::builtin::selectors {

class UniformRandomSelector final : public core::OperatorSelector {
public:
    explicit UniformRandomSelector(std::size_t number_of_operators)
        : random_index_(0, ValidateOperatorsCount(number_of_operators) - 1) {}

    [[nodiscard]] std::size_t Select(core::Rng& rng) override { return random_index_(rng); }

    void UpdateWeights(std::size_t /*operator_index*/, const core::OperatorReward& /*reward*/
                       ) noexcept override {}

    void ResetPeriod() noexcept override {}

private:
    std::uniform_int_distribution<std::size_t> random_index_;

    static std::size_t ValidateOperatorsCount(std::size_t number_of_operators) {
        if (number_of_operators == 0) {
            throw std::invalid_argument(
                "UniformRandomSelector: количество операторов должно быть "
                "больше нуля.");
        }
        return number_of_operators;
    }
};

}  // namespace alnslike::builtin::selectors