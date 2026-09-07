#pragma once
#include <alnslike/core/types.hpp>
#include <alnslike/tsp/coordinate.hpp>
#include <alnslike/tsp/details/candidate_search.hpp>
#include <vector>
#include <span>
#include <stdexcept>
#include <algorithm>

namespace alnslike::tsp {

class TspInstance {
public:
    static constexpr std::size_t kDefaultKNeighbors = 32;

    // При размере более 200 использовать матрицу не выгодно
    static constexpr std::size_t kDenseMatrixMaxNodes = 200;

    explicit TspInstance(std::vector<Coordinate> coordinates, std::size_t k_neighbors = kDefaultKNeighbors)
        : num_nodes_(coordinates.size())
        , coordinates_(std::move(coordinates))
        , k_neighbors_count_(SanitizeNeighboursCount(k_neighbors, num_nodes_)) {
        
        if (num_nodes_ == 0) {
            throw std::invalid_argument("TspInstance: список координат пуст.");
        }

        if (num_nodes_ <= kDenseMatrixMaxNodes) {
            BuildDenseMatrix();
            
            candidate_lists_ = detail::BuildNaiveCandidates(
                num_nodes_, k_neighbors_count_, 
                [this](NodeId u, NodeId v) { return Distance(u, v); }
            );
        } else {
            candidate_lists_ = detail::SpatialGridSearcher::BuildCandidates(coordinates_, k_neighbors_count_);
        }
    }

    TspInstance(std::size_t num_nodes, std::vector<core::Cost> matrix, std::size_t k_neighbors = kDefaultKNeighbors)
        : num_nodes_(num_nodes)
        , distance_matrix_(std::move(matrix))
        , has_distance_matrix_(true)
        , k_neighbors_count_(SanitizeNeighboursCount(k_neighbors, num_nodes_)) {
        
        if (distance_matrix_.size() != num_nodes_ * num_nodes_) {
            throw std::invalid_argument("TspInstance: размер матрицы не совпадает с N x N.");
        }
        
        candidate_lists_ = detail::BuildNaiveCandidates(
            num_nodes_, k_neighbors_count_, 
            [this](NodeId u, NodeId v) { return Distance(u, v); }
        );
    }

    [[nodiscard]] std::size_t GetSize() const noexcept { return num_nodes_; }
    [[nodiscard]] std::size_t GetKNeighborsCount() const noexcept { return k_neighbors_count_; }
    [[nodiscard]] const std::vector<Coordinate>& GetCoordinates() const noexcept { return coordinates_; }
    [[nodiscard]] bool HasDistanceMatrix() const noexcept { return has_distance_matrix_; }

    [[nodiscard]] core::Cost Distance(NodeId u, NodeId v) const noexcept {
        if (has_distance_matrix_) {
            return distance_matrix_[(u * num_nodes_) + v];
        }
        return coordinates_[u].DistanceTo(coordinates_[v]);
    }

    [[nodiscard]] std::span<const NodeId> NearestNeighbors(NodeId u) const noexcept {
        if (k_neighbors_count_ == 0) {return {};}
        return std::span<const NodeId>(&candidate_lists_[u * k_neighbors_count_], k_neighbors_count_);
    }

private:
    std::size_t num_nodes_{0};
    std::vector<Coordinate> coordinates_;
    std::vector<core::Cost> distance_matrix_;
    bool has_distance_matrix_{false};

    std::size_t k_neighbors_count_{0};
    std::vector<NodeId> candidate_lists_;

    static std::size_t SanitizeNeighboursCount(std::size_t neighbours_count, std::size_t nodes) noexcept {
        return std::min(neighbours_count, nodes > 1 ? nodes - 1 : 0);
    }

    // Для маленьких задач
    void BuildDenseMatrix() {
        distance_matrix_.resize(num_nodes_ * num_nodes_);
        for (NodeId u = 0; u < num_nodes_; ++u) {
            for (NodeId v = 0; v < num_nodes_; ++v) {
                distance_matrix_[(u * num_nodes_) + v] = (u == v) ? 0.0 : coordinates_[u].DistanceTo(coordinates_[v]);
            }
        }
        has_distance_matrix_ = true;
    }
};

} // namespace alnslike::tsp