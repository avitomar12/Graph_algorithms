#ifndef GRAPH_ALGORITHMS_ALGORITHM_HPP
#define GRAPH_ALGORITHMS_ALGORITHM_HPP

namespace graph_algorithms {

/**
 * @brief Base class for graph algorithms
 *
 * This class provides a base interface for graph algorithms.
 */
template <typename Graph>
class algorithm {
 public:
  /**
   * @brief Construct a new algorithm object
   *
   * @param graph The graph to operate on
   */
  algorithm(const Graph& graph) : graph_(graph) {}

  /**
   * @brief Run the algorithm
   *
   * This method should be overridden by derived classes to implement the specific algorithm.
   */
  virtual void run() = 0;

 protected:
  const Graph& graph_;  ///< The graph to operate on
};

}  // namespace graph_algorithms

#endif  // GRAPH_ALGORITHMS_ALGORITHM_HPP
