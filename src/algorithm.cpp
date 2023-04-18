#include "algorithm.hpp"

namespace graph_algorithms {

template <typename Graph>
algorithm<Graph>::algorithm(const Graph& graph) : graph_(graph) {}

template class algorithm<graph>;

}  // namespace graph_algorithms
