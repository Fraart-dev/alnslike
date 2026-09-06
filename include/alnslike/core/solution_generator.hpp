#pragma once
#include <alnslike/core/solution.hpp>
#include <alnslike/core/types.hpp>

namespace alnslike::core {

class SolutionGenerator {
public:
    virtual ~SolutionGenerator() = default;

    virtual void Generate(Solution& destination, Rng& rng) = 0;

    [[nodiscard]] virtual std::unique_ptr<Solution> CreateAndGenerate(Rng& rng) = 0;
};

}  // namespace alnslike::core