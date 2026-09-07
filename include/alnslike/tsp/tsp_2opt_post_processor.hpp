#pragma once

#include <alnslike/core/post_processor.hpp>
#include <alnslike/core/types.hpp>
#include <alnslike/tsp/tsp_solution.hpp>
#include <alnslike/tsp/tsp_opt_utils.hpp>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <cstdint>

namespace metaheuristic::tsp {
namespace core = alnslike::core;

// Пост-процессор, выполняющий жадный 2-opt по кандидатным спискам ближайших соседей.
// Работа ограничена заданным таймаутом.
class CandidateTwoOptPostProcessor final : public core::PostProcessor, private TspMutatorAccess {
public:
    struct Configuration {
        static constexpr std::size_t kDefaultTimeCheckMask = 1024 - 1;

        std::chrono::duration<double> timeout{1.0};
        std::size_t time_check_mask{kDefaultTimeCheckMask}; 
    };

    explicit CandidateTwoOptPostProcessor(Configuration config)
        : config_(ValidateConfiguration(config)) {}

    CandidateTwoOptPostProcessor()
        : CandidateTwoOptPostProcessor(Configuration{}) {}

    [[nodiscard]] core::SolutionDelta Optimize(core::Solution& solution, core::Rng& /*rng*/) noexcept override {
        auto& tsp_solution = static_cast<TspSolution&>(solution);
        auto mutator = TspMutatorAccess::GetMutator(tsp_solution);
        const auto* instance = tsp_solution.GetInstance();

        deadline_ = std::chrono::steady_clock::now() + 
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(config_.timeout);
        iteration_counter_ = 0;
        const std::size_t route_size = mutator.Route().size();

        core::Cost accumulated_delta = 0.0;
        bool use_full_search = false;

         while (!HasTimeoutExpired()) {
            const SearchOutcome outcome = TryCandidateImprovement(
                tsp_solution, mutator, instance, route_size, accumulated_delta);

            if (outcome == SearchOutcome::Timeout) {
                return {.objective_delta = accumulated_delta, .feasibility_delta = 0.0};
            }
            if (outcome == SearchOutcome::NoImprovement) {
                break;
            }
        }

        while (!HasTimeoutExpired()) {
            const SearchOutcome outcome = TryFullImprovement(
                tsp_solution, mutator, instance, route_size, accumulated_delta);

            if (outcome == SearchOutcome::Timeout) {
                return {.objective_delta = accumulated_delta, .feasibility_delta = 0.0};
            } 
            if (outcome == SearchOutcome::NoImprovement) {
                break;
            }
        }

        return {.objective_delta = accumulated_delta, .feasibility_delta = 0.0};
    }

private:

    enum class SearchOutcome : std::uint8_t {
        Improved,
        NoImprovement,
        Timeout
    };

    static Configuration ValidateConfiguration(Configuration config) {
        if (config.timeout <= std::chrono::duration<double>::zero()) {
            throw std::invalid_argument(
                "CandidateTwoOptPostProcessor: таймаут должен быть положительным.");
        }
        if ((config.time_check_mask & (config.time_check_mask + 1)) != 0) {
            throw std::invalid_argument(
                "CandidateTwoOptPostProcessor: маска проверки времени должна быть (2^N - 1).");
        }
        return config;
    }

    [[nodiscard]] bool HasTimeoutExpired() const noexcept {
        return (iteration_counter_ & config_.time_check_mask) == 0 &&
               std::chrono::steady_clock::now() >= deadline_;
    }

    bool AdvanceStepAndCheckTimeout() noexcept {
        ++iteration_counter_;
        return HasTimeoutExpired();
    }

    SearchOutcome TryCandidateImprovement(
    TspSolution& solution, auto& mutator,
    const TspInstance* instance, std::size_t route_size,
    core::Cost& accumulated_delta)
{
    for (std::size_t first_index = 0; first_index < route_size; ++first_index) {
        const NodeId first_node = mutator.Route()[first_index];
        const auto neighbors = instance->NearestNeighbors(first_node);

        for (const NodeId second_node : neighbors) {
            const std::size_t second_index = mutator.Positions()[second_node];
            const core::Cost delta = EvaluateTwoOpt(solution, first_index, second_index);

            if (delta < 0.0) {
                ApplyTwoOpt(mutator, first_index, second_index, delta);
                accumulated_delta += delta;
                return SearchOutcome::Improved;
            }

            if (AdvanceStepAndCheckTimeout()) {
                return SearchOutcome::Timeout;
            }
        }

        if (AdvanceStepAndCheckTimeout()) {
            return SearchOutcome::Timeout;
        }
    }

    return SearchOutcome::NoImprovement;
}

SearchOutcome TryFullImprovement(
    TspSolution& solution, auto& mutator,
    const TspInstance* instance, std::size_t route_size,
    core::Cost& accumulated_delta)
{
    for (std::size_t first_index = 0; first_index < route_size; ++first_index) {
        for (std::size_t second_index = first_index + 2; second_index < route_size; ++second_index) {
            if (first_index == 0 && second_index == route_size - 1) {
                continue;
            }

            const core::Cost delta = EvaluateTwoOpt(solution, first_index, second_index);
            if (delta < 0.0) {
                ApplyTwoOpt(mutator, first_index, second_index, delta);
                accumulated_delta += delta;
                return SearchOutcome::Improved;
            }

            if (AdvanceStepAndCheckTimeout()) {
                return SearchOutcome::Timeout;
            }
        }

        if (AdvanceStepAndCheckTimeout()) {
            return SearchOutcome::Timeout;
        }
    }

    return SearchOutcome::NoImprovement;
}

    Configuration config_;
    std::size_t iteration_counter_{0};
    std::chrono::steady_clock::time_point deadline_;
};

} // namespace metaheuristic::tsp