#pragma once

#include <alnslike/core/operator.hpp>
#include <alnslike/tsp/tsp_solution.hpp>
#include <alnslike/tsp/tsp_opt_utils.hpp>
#include <random>
#include <string_view>

namespace metaheuristic::tsp {
namespace core = alnslike::core;

class TwoOptOperatorBase : public core::LocalOperator, protected TspMutatorAccess {
protected:
    std::size_t cached_first_index_{0};
    std::size_t cached_second_index_{0};
    core::Cost cached_delta_{0.0};

public:
    void Finalize(core::Solution& base_solution, bool is_accepted) noexcept override {
        if (!is_accepted || cached_delta_ == 0.0) {
            return;
        }

        auto& solution = static_cast<TspSolution&>(base_solution);
        auto mutator = GetMutator(solution);
        
        ApplyTwoOpt(mutator, cached_first_index_, cached_second_index_, cached_delta_);
    }
};


class RandomTwoOptOperator final : public TwoOptOperatorBase {
private:
    std::size_t route_size_{0};
    std::uniform_int_distribution<std::size_t> node_distribution_;

public:
    explicit RandomTwoOptOperator(const TspInstance* instance)
        : route_size_(instance->GetSize())
        , node_distribution_(0, instance->GetSize() > 0 ? instance->GetSize() - 1 : 0) {}

    [[nodiscard]] std::string_view Name() const noexcept override { 
        return "TSP_RandomTwoOpt"; 
    }

    core::SolutionDelta Propose(core::Solution& base_solution, core::Rng& rng) noexcept override {
        auto& solution = static_cast<TspSolution&>(base_solution);
        auto mutator = GetMutator(solution);
        
        std::size_t first_index = node_distribution_(rng);
        std::size_t second_index = node_distribution_(rng);

        while (first_index == second_index || 
               second_index == (first_index + 1) % route_size_ || 
               first_index == (second_index + 1) % route_size_) {
            second_index = node_distribution_(rng);
        }

        cached_first_index_ = first_index;
        cached_second_index_ = second_index;
        
        cached_delta_ = EvaluateTwoOpt(solution, first_index, second_index);

        return core::SolutionDelta{
            .objective_delta = cached_delta_, 
            .feasibility_delta = 0.0
        };
    }
};

class CandidateRandomTwoOptOperator final : public TwoOptOperatorBase {
private:
    std::uniform_int_distribution<std::size_t> node_distribution_;
    std::uniform_int_distribution<std::size_t> candidate_distribution_;

public:
    explicit CandidateRandomTwoOptOperator(const TspInstance* instance)
        : node_distribution_(0, instance->GetSize() > 0 ? instance->GetSize() - 1 : 0)
        , candidate_distribution_(0, instance->GetKNeighborsCount() > 0 ? instance->GetKNeighborsCount() - 1 : 0) {}

    [[nodiscard]] std::string_view Name() const noexcept override { 
        return "TSP_CandidateRandomTwoOpt"; 
    }

    core::SolutionDelta Propose(core::Solution& base_solution, core::Rng& rng) noexcept override {
        auto& solution = static_cast<TspSolution&>(base_solution);
        const auto* instance = solution.GetInstance();
        auto mutator = GetMutator(solution);

        std::size_t first_index = node_distribution_(rng);
        const NodeId first_node = solution.GetRoute()[first_index];

        auto candidates = instance->NearestNeighbors(first_node);
        
        if (candidates.empty()) {
            std::size_t second_index = node_distribution_(rng);
            cached_first_index_ = first_index;
            cached_second_index_ = second_index;
            cached_delta_ = EvaluateTwoOpt(solution, first_index, second_index);
            
            return core::SolutionDelta{
                .objective_delta = cached_delta_, 
                .feasibility_delta = 0.0
            };
        }

        const NodeId second_node = candidates[candidate_distribution_(rng)];
        std::size_t second_index = solution.PositionOf(second_node);

        cached_first_index_ = first_index;
        cached_second_index_ = second_index;
        cached_delta_ = EvaluateTwoOpt(solution, first_index, second_index);

        return core::SolutionDelta{
            .objective_delta = cached_delta_, 
            .feasibility_delta = 0.0
        };
    }
};

} // namespace metaheuristic::tsp