#include <alnslike/builtin/criteria/simulated_annealing_criterion.hpp>
#include <alnslike/builtin/selectors/classic_roulette_selector.hpp>
#include <alnslike/core/local_step.hpp>
#include <alnslike/core/solver.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <string_view>
#include <vector>

namespace {

// Параметры задачи
constexpr int kInitialValue = 1000;

// Параметры солвера
constexpr std::size_t kMaxIterations = 10000;
constexpr std::chrono::seconds kTimeout{1};
constexpr uint64_t kSeed = 33;

// Параметры критерия имитации отжига
constexpr double kInitialTemperature = 1000.0;
constexpr double kCoolingRate = 0.999;

}  // namespace

class IntSolution final : public alnslike::core::Solution {
public:
    explicit IntSolution(int value) : value_(value) {}

    alnslike::core::Cost GetCost() const noexcept override { return value_ * value_; }
    alnslike::core::Cost GetFeasibilityViolation() const noexcept override { return 0.0; }

    std::unique_ptr<Solution> Clone() const override { return std::make_unique<IntSolution>(value_); }

    bool IsSolved() const noexcept override { return value_ == 0; }

    void CopyFrom(const Solution& other) override {
        const auto& other_int = static_cast<const IntSolution&>(other);
        value_ = other_int.value_;
    }

    int Value() const noexcept { return value_; }

    void Shift(int delta) noexcept { value_ += delta; }

private:
    int value_;
};

class ShiftOperator final : public alnslike::core::LocalOperator {
public:
    ShiftOperator(int delta, const char* name) : delta_(delta), name_(name) {}

    std::string_view Name() const noexcept override { return name_; }

    alnslike::core::SolutionDelta Propose(alnslike::core::Solution& solution,
                                          alnslike::core::Rng& /*rng*/) noexcept override {
        const auto& sol = static_cast<const IntSolution&>(solution);

        int current = sol.Value();
        int proposed = current + delta_;

        return {.objective_delta = static_cast<double>((proposed * proposed) - (current * current)),
                .feasibility_delta = 0.0};
    }

    void Finalize(alnslike::core::Solution& solution, bool is_accepted) noexcept override {
        if (is_accepted) {
            static_cast<IntSolution&>(solution).Shift(delta_);
        }
    }

private:
    int delta_;
    std::string name_;
};

struct DummyMoveCache final : alnslike::core::MoveCache {
    void Clear() noexcept override {}
};

int main() {
    auto initial = std::make_unique<IntSolution>(kInitialValue);

    std::vector<std::unique_ptr<alnslike::core::LocalOperator>> operators;
    operators.push_back(std::make_unique<ShiftOperator>(1, "Increment operator"));
    operators.push_back(std::make_unique<ShiftOperator>(-1, "Decrement operator"));

    auto selector = std::make_unique<alnslike::builtin::selectors::ClassicRouletteSelector>(operators.size());

    auto step = std::make_unique<alnslike::core::LocalSearchStep>(std::move(selector), std::move(operators));

    alnslike::builtin::criteria::SimulatedAnnealingCriterion::Configuration sa_config = {
        .initial_temperature = kInitialTemperature, .cooling_rate = kCoolingRate};
    auto sa_criterion = std::make_unique<alnslike::builtin::criteria::SimulatedAnnealingCriterion>(sa_config);

    auto solver = alnslike::core::SolverBuilder{}
                      .SetMaxIterations(kMaxIterations)
                      .SetTimeout(kTimeout)
                      .SetSeed(kSeed)
                      .SetInitialSolution(std::move(initial))
                      .SetSearchStep(std::move(step))
                      .SetAcceptanceCriterion(std::move(sa_criterion))
                      .SetMoveCache(std::make_unique<DummyMoveCache>())
                      .Build();

    solver->Run();

    const auto& best = solver->BestSolution();
    std::cout << "Best value: " << static_cast<const IntSolution&>(best).Value() << "\n";
}