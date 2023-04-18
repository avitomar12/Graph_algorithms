#include "graph.hpp"

namespace graph_algorithms {

template <typename Vertex, typename Edge>
void graph<Vertex, Edge>::add_vertex(const Vertex& vertex) {
  if (vertices_.find(vertex.id) != vertices_.end()) {
    throw std::invalid_argument("Vertex already exists: " + vertex.id);
  }
  vertices_[vertex.id] = vertex;
}

template <typename Vertex, typename Edge>
void graph<Vertex, Edge>::add_edge(const Vertex& from, const Vertex& to, const Edge& edge) {
  add_vertex(from);
  add_vertex(to);
  adj_list_[from.id].emplace_back(to.id, edge);
}

template <typename Vertex, typename Edge>
const Vertex& graph<Vertex, Edge>::get_vertex(const std::string& id) const {
  if (vertices_.find(id) == vertices_.end()) {
    throw std::invalid_argument("Vertex not found: " + id);
  }
  return vertices_.at(id);
}

template <typename Vertex, typename Edge>
const std::vector<std::pair<std::string, Edge>>& graph<Vertex, Edge>::get_adj_list(const std::string& id) const {
  if (adj_list_.find(id) == adj_list_.end()) {
    throw std::invalid_argument("Vertex not found: " + id);
  }
  return adj_list_.at(id);
}

template <typename Vertex, typename Edge>
const std::unordered_map<std::string, Vertex>& graph<Vertex, Edge>::vertices() const {
  return vertices_;
}

template <typename Vertex, typename Edge>
const std::unordered_map<std::string, std::vector<std::pair<std::string, Edge>>>& graph<Vertex, Edge>::adj_list() const {
  return adj_list_;
}

template class graph<vertex, int>;

}  // namespace graph_algorithms
