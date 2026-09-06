#pragma once
#include <alnslike/core/types.hpp>
#include <cstddef>

namespace alnslike::core {

class OperatorSelector {
public:
    virtual ~OperatorSelector() = default;

    virtual std::size_t Select(Rng& rng) = 0;

    virtual void UpdateWeights(std::size_t operator_index, const OperatorReward& reward) noexcept = 0;

    virtual void ResetPeriod() noexcept = 0;
};

}  // namespace alnslike::core