#include "shortest_path_algorithm.hpp"

namespace graph_algorithms {

template <typename Graph>
shortest_path_algorithm<Graph>::shortest_path_algorithm(const Graph& graph) : algorithm<Graph>(graph) {}

template <typename Graph>
std::vector<typename Graph::vertex_type> shortest_path_algorithm<Graph>::get_shortest_path(const typename Graph::vertex_type& source,
                                                                                          const typename Graph::vertex_type& target) {
  // Initialize priority queue and visited set
  std::priority_queue<node, std::vector<node>, std::greater<node>> pq;
  std::unordered_set<std::string> visited;

  // Initialize distances and previous vertex map
  std::unordered_map<std::string, int> dist;
  std::unordered_map<std::string, std::string> prev;
  for (const auto& [id, vertex] : this->graph_.vertices()) {
    dist[id] = std::numeric_limits<int>::max();
    prev[id] = "";
  }

  // Add source vertex to queue and set its distance to 0
  pq.emplace(source.id, 0);
  dist[source.id] = 0;

  // Dijkstra's algorithm
  while (!pq.empty()) {
    // Get vertex with smallest distance
    const auto [curr_id, curr_dist] = pq.top();
    pq.pop();
    if (visited.count(curr_id) > 0) {
      continue;
    }
    visited.emplace(curr_id);

    // Check if target has been reached
    if (curr_id == target.id) {
      break;
    }

    // Update distances and previous vertex for neighboring vertices
    for (const auto& [next_id, edge] : this->graph_.get_adj_list(curr_id)) {
      const auto next_dist = curr_dist + edge;
      if (next_dist < dist[next_id]) {
        dist[next_id] = next_dist;
        prev[next_id] = curr_id;
        pq.emplace(next_id, next_dist);
      }
    }
  }

  // Reconstruct shortest path by following previous vertex map
  std::vector<typename Graph::vertex_type> path;
  for (auto curr_id = target.id; curr_id != ""; curr_id = prev[curr_id]) {
    path.emplace_back(this->graph_.get_vertex(curr_id));
  }
  std::reverse(path.begin(), path.end());

  return path;
}

template class shortest_path_algorithm<graph<vertex, int>>;

}  // namespace graph_algorithms
