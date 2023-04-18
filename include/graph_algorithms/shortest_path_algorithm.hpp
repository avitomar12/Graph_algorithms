#ifndef GRAPH_ALGORITHMS_SHORTEST_PATH_ALGORITHM_HPP
#define GRAPH_ALGORITHMS_SHORTEST_PATH_ALGORITHM_HPP

#include <limits>
#include <queue>
#include <unordered_map>

#include "algorithm.hpp"
#include "graph.hpp"

namespace graph_algorithms {

/**
 * @brief Base class for shortest path algorithms
 *
 * This class provides a base interface for shortest path algorithms.
 */
template <typename Graph, typename Vertex>
class shortest_path_algorithm : public algorithm<Graph> {
 public:
  using algorithm<Graph>::algorithm;

  /**
   * @brief Get the shortest path from the source vertex to a target vertex
   *
   * @param target The target vertex
   * @return std::vector<Vertex> The shortest path
   */
  std::vector<Vertex> shortest_path(const Vertex& target) const {
    std::vector<Vertex> path;
    if (dist_.find(target) == dist_.end()) {
      return path;
    }
    Vertex curr = target;
    while (curr != source_) {
      path.push_back(curr);
      curr = prev_.at(curr);
    }
    path.push_back(curr);
    std::reverse(path.begin(), path.end());
    return path;
  }

 protected:
  Vertex source_;  ///< The source vertex for the shortest paths
  std::unordered_map<Vertex, int> dist_;  ///< The distances from the source vertex to each vertex
  std::unordered_map<Vertex, Vertex> prev_;  ///< The previous vertex in the shortest path to each vertex

  /**
   * @brief Initialize the distances and previous vertices
   *
   * @param source The source vertex
   */
  void initialize(const Vertex& source) {
    source_ = source;
    for (const auto& [id, adj_list] : this->graph_.adj_list()) {
      dist_[id] = std::numeric_limits<int>::max();
      prev_[id] = Vertex();
    }
    dist_[source] = 0;
  }

  /**
   * @brief Relax an edge
   *
   * @param from The source vertex of the edge
   * @param to The target vertex of the edge
   * @param weight The weight of the edge
   * @return true if the distance to the target vertex was updated, false otherwise
   */
  bool relax(const Vertex& from, const Vertex& to, int weight) {
    if (dist_[from] + weight < dist_[to]) {
      dist_[to] = dist_[from] + weight;
      prev_[to] = from;
      return true;
    }
    return false;
  }
};

}  // namespace graph_algorithms

#endif  // GRAPH_ALGORITHMS_SHORTEST_PATH_ALGORITHM_HPP
