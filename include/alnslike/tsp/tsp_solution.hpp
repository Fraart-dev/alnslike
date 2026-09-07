#pragma once
#include <alnslike/core/solution.hpp>
#include <alnslike/core/types.hpp>
#include <alnslike/tsp/tsp_instance.hpp>
#include <vector>

namespace metaheuristic::tsp {
    namespace core = alnslike::core;

class TspSolution final : public core::Solution {

    friend class TspMutatorAccess;

public:
    explicit TspSolution(const TspInstance* instance, std::vector<NodeId> initial_route)
        : instance_(instance)
        , route_{std::move(initial_route)}
        , position_in_route_(instance->GetSize()) {
            for (std::size_t i = 0; i < route_.size(); ++i) {
                position_in_route_[route_[i]] = i;
            }

            RecalculateCost();
        }


        core::Cost GetCost() const noexcept override {return total_cost_;}

        core::Cost GetFeasibilityViolation() const noexcept override {return 0.0;}

        std::unique_ptr<core::Solution> Clone() const override {
            return std::make_unique<TspSolution>(*this);
        }

        void CopyFrom(const core::Solution& other) override {
            const auto& other_tsp = static_cast<const TspSolution&>(other);
            instance_ = other_tsp.instance_;
            route_ = other_tsp.route_;
            position_in_route_ = other_tsp.position_in_route_;
            total_cost_ = other_tsp.total_cost_;
        }

        const std::vector<NodeId>& GetRoute() const noexcept {return route_;}
        std::size_t PositionOf(NodeId node_id) const noexcept {return position_in_route_[node_id];}
        const TspInstance* GetInstance() const noexcept { return instance_; }


        void RecalculateCost() noexcept {
            total_cost_ = 0.0;
            const std::size_t node_count = route_.size();
            
            for (std::size_t i = 0; i < node_count; ++i) {
                total_cost_ += instance_->Distance(route_[i], route_[(i + 1) % node_count]);
            }
        }

        class Mutator {
            friend class TspSolution;
        public: 
            std::vector<NodeId>& Route() noexcept { return solution_.route_; }
            std::vector<std::size_t>& Positions() noexcept { return solution_.position_in_route_; }
            core::Cost& Cost() noexcept { return solution_.total_cost_; }

        private:
            TspSolution& solution_;

            explicit Mutator(TspSolution& solution) noexcept : solution_(solution) {}
        };

private:

    Mutator GetMutator() noexcept {
        return Mutator(*this);
    }

    const TspInstance* instance_;
    std::vector<NodeId> route_;
    std::vector<std::size_t> position_in_route_;
    core::Cost total_cost_{0.0};

};

// Используется в операторах для прямого доступа к полям solution
// При изменении решения необходимо самостоятельно синхронизировать route_, position_in_route_, total_cost_ и следить за валидностью состояния
class TspMutatorAccess {
protected:
    static TspSolution::Mutator GetMutator(TspSolution& solution) noexcept {return solution.GetMutator();};
};

}