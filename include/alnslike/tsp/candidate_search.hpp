#pragma once
#include <alnslike/tsp/coordinate.hpp>
#include <cstddef>
#include <vector>
#include <cmath>
#include <algorithm>

namespace metaheuristic::tsp::detail {

struct NeighborInfo {
    double distance_squared;
    NodeId id;
    bool operator<(const NeighborInfo& other) const { 
        return distance_squared < other.distance_squared; 
    }
};

class SpatialGridSearcher {
public:
    using GridIndex = int; 
    using GridOffset = int;

    static std::vector<NodeId> BuildCandidates(
        const std::vector<Coordinate>& coordinates, 
        std::size_t neighbor_count
    ) {
        if (neighbor_count == 0 || coordinates.empty()) {return {};}

        SpatialGridSearcher searcher(coordinates, neighbor_count);
        return searcher.RunSearch();
    }

private:
    const std::vector<Coordinate>& coordinates_;
    std::size_t neighbor_count_;
    std::size_t number_of_nodes_;

    double min_x_{0.0}, min_y_{0.0};
    double cell_width_{1.0}, cell_height_{1.0};
    GridIndex grid_width_{1}, grid_height_{1};
    std::vector<std::vector<NodeId>> grid_;

    SpatialGridSearcher(const std::vector<Coordinate>& coordinates, std::size_t neighbor_count)
        : coordinates_(coordinates)
        , neighbor_count_(neighbor_count)
        , number_of_nodes_(coordinates.size()) {
        
        ComputeBounds();
        CalculateGridDimensions();
        PopulateGrid();
    }

    void ComputeBounds() {
        min_x_ = coordinates_[0].x;
        min_y_ = coordinates_[0].y;
        double max_x = min_x_;
        double max_y = min_y_;

        for (const auto& coordinate : coordinates_) {
            min_x_ = std::min(min_x_, coordinate.x); 
            max_x = std::max(max_x, coordinate.x);
            
            min_y_ = std::min(min_y_, coordinate.y); 
            max_y = std::max(max_y, coordinate.y);
        }
        
        cell_width_ = std::max(max_x - min_x_, 1.0);
        cell_height_ = std::max(max_y - min_y_, 1.0);
    }

    void CalculateGridDimensions() {
        constexpr double kTargetNodesPerCell = 4.0;
        const double target_cells_count = static_cast<double>(number_of_nodes_) / kTargetNodesPerCell;
        const double aspect_ratio = cell_width_ / cell_height_;

        grid_width_ = std::max(1, static_cast<int>(std::sqrt(target_cells_count * aspect_ratio)));
        grid_height_ = std::max(1, static_cast<int>(std::sqrt(target_cells_count / aspect_ratio)));

        cell_width_ /= static_cast<double>(grid_width_);
        cell_height_ /= static_cast<double>(grid_height_);
    }

    void PopulateGrid() {
        grid_.resize(static_cast<std::size_t>(grid_width_) * grid_height_);
        for (NodeId node_index = 0; node_index < number_of_nodes_; ++node_index) {
            grid_[GetCellLinearIndex(coordinates_[node_index].x, coordinates_[node_index].y)].push_back(node_index);
        }
    }

    std::vector<NodeId> RunSearch() {
        std::vector<NodeId> candidate_lists(number_of_nodes_ * neighbor_count_);
        std::vector<NeighborInfo> local_candidates;
        local_candidates.reserve(neighbor_count_ * 2);

        for (NodeId current_node = 0; current_node < number_of_nodes_; ++current_node) {
            local_candidates.clear();
            
            const GridIndex cell_x_index = GetCellColumnIndex(coordinates_[current_node].x);
            const GridIndex cell_y_index = GetCellRowIndex(coordinates_[current_node].y);
            
            const GridOffset max_radius = CalculateMaximumSearchRadius(cell_x_index, cell_y_index);
            GridOffset search_radius = 0;

            while (search_radius <= max_radius) {
                ExploreRing(current_node, cell_x_index, cell_y_index, search_radius, local_candidates);

                if (local_candidates.size() >= neighbor_count_) {                    
                    std::partial_sort(
                        local_candidates.begin(), 
                        local_candidates.begin() + static_cast<std::ptrdiff_t>(neighbor_count_), 
                        local_candidates.end()
                    );
                    
                    if (IsNextRingFurtherThanFurthestCandidate(search_radius, local_candidates[neighbor_count_ - 1].distance_squared)) {
                        break;
                    }
                }
                ++search_radius;
            }

            const std::size_t offset = current_node * neighbor_count_;
            for (std::size_t candidate_index = 0; candidate_index < neighbor_count_; ++candidate_index) {
                candidate_lists[offset + candidate_index] = local_candidates[candidate_index].id;
            }
        }
        return candidate_lists;
    }

void ExploreRing(NodeId current_node, GridIndex center_cell_index_x, GridIndex center_cell_index_y, GridOffset search_radius, std::vector<NeighborInfo>& candidates) {

        auto process_cell = [&](GridIndex cell_index_x, GridIndex cell_index_y) {
            for (NodeId candidate_node : grid_[(cell_index_y * grid_width_) + cell_index_x]) {
                if (current_node != candidate_node) {
                    double distance_squared = coordinates_[current_node].DistanceSquared(coordinates_[candidate_node]);
                    candidates.push_back({distance_squared, candidate_node});
                }
            }
        };

        if (search_radius == 0) {
            process_cell(center_cell_index_x, center_cell_index_y);
            return;
        }

        const GridIndex ring_min_x = center_cell_index_x - search_radius;
        const GridIndex ring_max_x = center_cell_index_x + search_radius;
        const GridIndex ring_min_y = center_cell_index_y - search_radius;
        const GridIndex ring_max_y = center_cell_index_y + search_radius;

        const GridIndex valid_min_x = std::max(0, ring_min_x);
        const GridIndex valid_max_x = std::min(grid_width_ - 1, ring_max_x);
        const GridIndex valid_min_y = std::max(0, ring_min_y);
        const GridIndex valid_max_y = std::min(grid_height_ - 1, ring_max_y);

        // (y = ring_min_y)
        if (ring_min_y >= 0 && ring_min_y < grid_height_) {
            for (GridIndex x = valid_min_x; x <= valid_max_x; ++x) {
                process_cell(x, ring_min_y);
            }
        }

        // (y = ring_max_y)
        if (ring_max_y >= 0 && ring_max_y < grid_height_ && ring_max_y != ring_min_y) {
            for (GridIndex x = valid_min_x; x <= valid_max_x; ++x) {
                process_cell(x, ring_max_y);
            }
        }

        // (x = ring_min_x)
        if (ring_min_x >= 0 && ring_min_x < grid_width_) {

            const GridIndex start_y = std::max(0, ring_min_y + 1);
            const GridIndex end_y = std::min(grid_height_ - 1, ring_max_y - 1);

            for (GridIndex y = start_y; y <= end_y; ++y) {
                process_cell(ring_min_x, y);
            }
        }

        // (x = ring_max_x)
        if (ring_max_x >= 0 && ring_max_x < grid_width_ && ring_max_x != ring_min_x) {

            const GridIndex start_y = std::max(0, ring_min_y + 1);
            const GridIndex end_y = std::min(grid_height_ - 1, ring_max_y - 1);

            for (GridIndex y = start_y; y <= end_y; ++y) {
                process_cell(ring_max_x, y);
            }
        }
    }
    
    // У круга и квадрата разное расстояние до края
    bool IsNextRingFurtherThanFurthestCandidate(GridOffset current_radius, double maximum_candidate_distance_squared) const {
        double distance_to_next_ring = static_cast<double>(current_radius) * std::min(cell_width_, cell_height_);

        return (distance_to_next_ring * distance_to_next_ring) >= maximum_candidate_distance_squared;
    }
    
    GridIndex GetCellColumnIndex(double x_coordinate) const { 
        GridIndex cell_x = static_cast<GridIndex>((x_coordinate - min_x_) / cell_width_);
        
        if (cell_x < 0) {
            return 0;
        }
        if (cell_x >= grid_width_) {
            return grid_width_ - 1;
        }
        
        return cell_x;
    }
    
    GridIndex GetCellRowIndex(double y_coordinate) const { 
        GridIndex cell_y = static_cast<GridIndex>((y_coordinate - min_y_) / cell_height_);
        
        if (cell_y < 0) {
            return 0;
        }
        if (cell_y >= grid_height_) {
            return grid_height_ - 1;
        }
        
        return cell_y;
    }
    
    GridIndex GetCellLinearIndex(double x_coordinate, double y_coordinate) const { 
        return (GetCellRowIndex(y_coordinate) * grid_width_) + GetCellColumnIndex(x_coordinate); 
    }

    GridIndex CalculateMaximumSearchRadius(GridIndex cell_x_index, GridIndex cell_y_index) const {
        const GridOffset distance_to_left_edge = cell_x_index;
        const GridOffset distance_to_right_edge = grid_width_ - 1 - cell_x_index;
        const GridOffset distance_to_top_edge = cell_y_index;
        const GridOffset distance_to_bottom_edge = grid_height_ - 1 - cell_y_index;

        return std::max({
            distance_to_left_edge, 
            distance_to_right_edge, 
            distance_to_top_edge, 
            distance_to_bottom_edge
        });
    }
};

// Функция для наивного перебора 
template <typename DistanceAccessor>
std::vector<NodeId> BuildNaiveCandidates(
    std::size_t number_of_nodes, 
    std::size_t neighbor_count, 
    DistanceAccessor&& distance_accessor
) {
    if (neighbor_count == 0 || number_of_nodes <= 1) {return {};}

    std::vector<NodeId> candidate_lists(number_of_nodes * neighbor_count);
    std::vector<NeighborInfo> candidates;
    candidates.reserve(number_of_nodes);

    for (NodeId current_node = 0; current_node < number_of_nodes; ++current_node) {
        candidates.clear();
        for (NodeId candidate_node = 0; candidate_node < number_of_nodes; ++candidate_node) {
            if (current_node != candidate_node) {
                double distance = distance_accessor(current_node, candidate_node);
                candidates.push_back({distance * distance, candidate_node});
            }
        }
        
        std::partial_sort(candidates.begin(), candidates.begin() + static_cast<std::ptrdiff_t>(neighbor_count), candidates.end());
        
        const std::size_t offset = current_node * neighbor_count;
        for (std::size_t index = 0; index < neighbor_count; ++index) {
            candidate_lists[offset + index] = candidates[index].id;
        }
    }
    return candidate_lists;
}

} // namespace metaheuristic::tsp::detail