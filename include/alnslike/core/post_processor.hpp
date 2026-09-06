#pragma once
#include <alnslike/core/solution.hpp>
#include <alnslike/core/types.hpp>

namespace alnslike::core {

// Улучшает лучшее найденное решение, вызывается в конце работы солвера
class PostProcessor {
public:
    virtual ~PostProcessor() = default;

    virtual SolutionDelta Optimize(Solution& solution, Rng& rng) noexcept = 0;
};

}  // namespace alnslike::core