#pragma once

#include <alnslike/core/solution_generator.hpp>
#include <alnslike/core/types.hpp>
#include <alnslike/tsp/tsp_instance.hpp>
#include <alnslike/tsp/tsp_solution.hpp>

#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <vector>

namespace metaheuristic::tsp {
    
namespace core = alnslike::core;

class NearestNeighborGenerator final : public core::SolutionGenerator, protected TspMutatorAccess {
public:
    explicit NearestNeighborGenerator(const TspInstance* instance)
        : instance_(ValidateInstance(instance)),
          start_distribution_(0, instance_->GetSize() - 1) {}

    void Generate(core::Solution& destination, core::Rng& rng) noexcept override {
        auto& tsp_solution = static_cast<TspSolution&>(destination);
        auto mutator = TspMutatorAccess::GetMutator(tsp_solution);

        std::vector<NodeId> route = GenerateRoute(rng);

        mutator.Route() = std::move(route);
        mutator.Positions().assign(mutator.Route().size(), 0);
        mutator.Cost() = 0.0;

        for (std::size_t index = 0; index < mutator.Route().size(); ++index) {
            mutator.Positions()[mutator.Route()[index]] = index;
        }

        const std::size_t route_size = mutator.Route().size();
        for (std::size_t index = 0; index < route_size; ++index) {
            const std::size_t next_index = (index + 1) % route_size;
            mutator.Cost() += instance_->Distance(mutator.Route()[index], mutator.Route()[next_index]);
        }
    }

    [[nodiscard]] std::unique_ptr<core::Solution> CreateAndGenerate(core::Rng& rng) noexcept override {
        return std::make_unique<TspSolution>(instance_, GenerateRoute(rng));
    }

private:
    static const TspInstance* ValidateInstance(const TspInstance* instance) {
        if (instance == nullptr) {
            throw std::invalid_argument("NearestNeighborGenerator: instance не должен быть nullptr.");
        }
        if (instance->GetSize() == 0) {
            throw std::invalid_argument("NearestNeighborGenerator: instance не должен иметь нулевой размер.");
        }
        return instance;
    }

    [[nodiscard]] std::vector<NodeId> GenerateRoute(core::Rng& rng) {
        const std::size_t node_count = instance_->GetSize();
        std::vector<bool> visited(node_count, false);
        std::vector<NodeId> route;
        route.reserve(node_count);

        NodeId current_node = start_distribution_(rng);
        visited[current_node] = true;
        route.push_back(current_node);

        for (std::size_t step = 1; step < node_count; ++step) {
            NodeId best_candidate = kInvalidNode;
            core::Cost best_distance = std::numeric_limits<core::Cost>::max();

            for (const NodeId candidate : instance_->NearestNeighbors(current_node)) {
                if (!visited[candidate]) {
                    const core::Cost distance = instance_->Distance(current_node, candidate);
                    if (distance < best_distance) {
                        best_distance = distance;
                        best_candidate = candidate;
                    }
                }
            }

            if (best_candidate == kInvalidNode) {
                for (NodeId candidate = 0; candidate < node_count; ++candidate) {
                    if (!visited[candidate]) {
                        const core::Cost distance = instance_->Distance(current_node, candidate);
                        if (distance < best_distance) {
                            best_distance = distance;
                            best_candidate = candidate;
                        }
                    }
                }
            }

            current_node = best_candidate;
            visited[current_node] = true;
            route.push_back(current_node);
        }

        return route;
    }

    const TspInstance* instance_;
    std::uniform_int_distribution<std::size_t> start_distribution_;
};

} // namespace metaheuristic::tsp