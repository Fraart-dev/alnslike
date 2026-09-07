#pragma once

#include <alnslike/core/operator.hpp>
#include <alnslike/tsp/tsp_solution.hpp>
#include <alnslike/tsp/tsp_opt_utils.hpp>
#include <algorithm>
#include <cstddef>
#include <random>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace alnslike::tsp {

class ThreeOptOperatorBase : public core::LocalOperator, protected TspMutatorAccess {
protected:
    std::size_t cached_first_index_{0};
    std::size_t cached_second_index_{0};
    std::size_t cached_third_index_{0};
    ThreeOptMove cached_move_{};

public:
    void Finalize(core::Solution& base_solution, bool is_accepted) noexcept override {
        if (!is_accepted || cached_move_.reconnection_case == ThreeOptReconnectionCase::None) {
            return;
        }

        auto& solution = static_cast<TspSolution&>(base_solution);
        auto mutator = TspMutatorAccess::GetMutator(solution);

        ApplyThreeOpt(
            mutator,
            cached_first_index_,
            cached_second_index_,
            cached_third_index_,
            cached_move_
        );
    }
};


template <bool ExcludeTwoOptMoves>
class GenericRandomThreeOptOperator final : public ThreeOptOperatorBase {
    static constexpr size_t kMinInstanceSizeForThreeOpt = 6;
private:
    std::size_t route_size_{0};
    std::uniform_int_distribution<std::size_t> node_distribution_;

    static const TspInstance* ValidateInstance(const TspInstance* instance) {
        if (instance == nullptr || instance->GetSize() < kMinInstanceSizeForThreeOpt) {
            throw std::invalid_argument(
                "GenericRandomThreeOptOperator: instance должен содержать минимум 6 вершин.");
        }
        return instance;
    }

public:
    explicit GenericRandomThreeOptOperator(const TspInstance* instance)
        : route_size_(instance->GetSize()),
          node_distribution_(0, instance->GetSize() - 1) {
        ValidateInstance(instance);
    }

    [[nodiscard]] std::string_view Name() const noexcept override {
        return ExcludeTwoOptMoves ? "TSP_PureRandomThreeOpt" : "TSP_RandomThreeOpt";
    }

    core::SolutionDelta Propose(core::Solution& base_solution, core::Rng& rng) noexcept override {
        auto& solution = static_cast<TspSolution&>(base_solution);

        std::size_t first_index;
        std::size_t second_index;
        std::size_t third_index;

        do {
            first_index = node_distribution_(rng);
            second_index = node_distribution_(rng);
            third_index = node_distribution_(rng);

            if (first_index > second_index) {std::swap(first_index, second_index);}
            if (second_index > third_index) {std::swap(second_index, third_index);}
            if (first_index > second_index) {std::swap(first_index, second_index);}

        } while (first_index + 1 >= second_index ||
                 second_index + 1 >= third_index ||
                 (first_index == 0 && third_index == route_size_ - 1));

        cached_first_index_ = first_index;
        cached_second_index_ = second_index;
        cached_third_index_ = third_index;

        cached_move_ = evaluate_three_opt<ExcludeTwoOptMoves>(solution, first_index, second_index, third_index);

        return core::SolutionDelta{
            .objective_delta = cached_move_.delta,
            .feasibility_delta = 0.0
        };
    }
};


template <bool ExcludeTwoOptMoves>
class GenericCandidateThreeOptOperator final : public ThreeOptOperatorBase {
        static constexpr size_t kMinInstanceSizeForThreeOpt = 6;
private:
    std::uniform_int_distribution<std::size_t> node_distribution_;
    std::uniform_int_distribution<std::size_t> candidate_distribution_;

    static const TspInstance* ValidateInstance(const TspInstance* instance) {
        if (instance == nullptr || instance->GetSize() < kMinInstanceSizeForThreeOpt ||
            instance->GetKNeighborsCount() < 2) {
            throw std::invalid_argument(
                "GenericCandidateThreeOptOperator: instance должен содержать минимум 6 вершин и 2 кандидата.");
        }
        return instance;
    }

public:
    explicit GenericCandidateThreeOptOperator(const TspInstance* instance)
        : node_distribution_(0, instance->GetSize() - 1),
          candidate_distribution_(0, instance->GetKNeighborsCount()- 1) {
        ValidateInstance(instance);
    }

    [[nodiscard]] std::string_view Name() const noexcept override {
        return ExcludeTwoOptMoves ? "TSP_PureCandidateThreeOpt" : "TSP_CandidateThreeOpt";
    }

    core::SolutionDelta Propose(core::Solution& base_solution, core::Rng& rng) noexcept override {
        auto& solution = static_cast<TspSolution&>(base_solution);
        const auto* instance = solution.GetInstance();
        auto mutator = TspMutatorAccess::GetMutator(solution);

        std::size_t first_index = node_distribution_(rng);
        const NodeId first_node = mutator.Route()[first_index];

        const auto candidates = instance->NearestNeighbors(first_node);
        if (candidates.size() < 2) {
            cached_move_ = ThreeOptMove{0.0, ThreeOptReconnectionCase::None};
            return core::SolutionDelta{0.0, 0.0};
        }

        NodeId second_node = candidates[candidate_distribution_(rng)];
        NodeId third_node = candidates[candidate_distribution_(rng)];

        while (third_node == second_node) {
            third_node = candidates[candidate_distribution_(rng)];
        }

        std::size_t second_index = solution.PositionOf(second_node);
        std::size_t third_index = solution.PositionOf(third_node);

        if (first_index > second_index) {std::swap(first_index, second_index);}
        if (second_index > third_index) {std::swap(second_index, third_index);}
        if (first_index > second_index) {std::swap(first_index, second_index);}

        cached_first_index_ = first_index;
        cached_second_index_ = second_index;
        cached_third_index_ = third_index;

        cached_move_ = evaluate_three_opt<ExcludeTwoOptMoves>(solution, first_index, second_index, third_index);

        return core::SolutionDelta{
            .objective_delta = cached_move_.delta,
            .feasibility_delta = 0.0
        };
    }
};


using RandomThreeOptOperator         = GenericRandomThreeOptOperator<false>;
using PureRandomThreeOptOperator     = GenericRandomThreeOptOperator<true>;

using CandidateThreeOptOperator      = GenericCandidateThreeOptOperator<false>;
using PureCandidateThreeOptOperator  = GenericCandidateThreeOptOperator<true>;

} // namespace alnslike::tsp