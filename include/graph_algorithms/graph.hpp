#ifndef GRAPH_ALGORITHMS_GRAPH_HPP
#define GRAPH_ALGORITHMS_GRAPH_HPP

#include <vector>
#include <unordered_map>

namespace graph_algorithms {

/**
 * @brief A directed graph
 *
 * This class represents a directed graph, where each vertex is identified by a unique identifier of type T.
 */
template <typename T>
class graph {
 public:
  /**
   * @brief Add a vertex to the graph
   *
   * @param id The identifier of the vertex
   */
  void add_vertex(const T& id) {
    if (adj_list_.find(id) == adj_list_.end()) {
      adj_list_[id] = std::vector<std::pair<T, int>>();
    }
  }

  /**
   * @brief Add an edge to the graph
   *
   * @param from The identifier of the vertex the edge comes from
   * @param to The identifier of the vertex the edge goes to
   * @param weight The weight of the edge
   */
  void add_edge(const T& from, const T& to, int weight) {
    adj_list_[from].push_back({to, weight});
  }

  /**
   * @brief Get the adjacency list for a vertex
   *
   * @param id The identifier of the vertex
   * @return const std::vector<std::pair<T, int>>& The adjacency list for the vertex
   */
  const std::vector<std::pair<T, int>>& adj_list(const T& id) const {
    return adj_list_.at(id);
  }

 private:
  std::unordered_map<T, std::vector<std::pair<T, int>>> adj_list_;  ///< The adjacency list of the graph
};

}  // namespace graph_algorithms

#endif  // GRAPH_ALGORITHMS_GRAPH_HPP
