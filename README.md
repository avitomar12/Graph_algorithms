# Graph Algorithms Library

The Graph Algorithms Library is a C++ library for working with graphs and implementing various graph algorithms. This library is designed to be easy to use and highly modular, allowing developers to easily incorporate it into their own projects.

## Installation

To use this library in your own C++ project, you can include the header files from the `include` directory and link against the `graph_algorithms` library.

### Prerequisites

- C++11 or later
- CMake 3.16 or later

### Build Instructions

To build the library from source, follow these steps:

1. Clone the repository:
git clone https://github.com/your_username/graph_algorithms.git

2. Change into the `graph_algorithms` directory:

cd graph_algorithms


3. Create a build directory and change into it:
mkdir build
cd build

4. Generate the build files using CMake:


cmake ..




## Usage

To use the library in your own C++ project, include the necessary header files from the `include` directory and link against the `graph_algorithms` library. For example:

```c++
#include <graph_algorithms/graph.hpp>
#include <graph_algorithms/shortest_path_algorithm.hpp>

int main() {
// Create a graph
graph_algorithms::graph<int> g;

// Add vertices and edges
g.add_vertex(1);
g.add_vertex(2);
g.add_vertex(3);
g.add_edge(1, 2, 5);
g.add_edge(2, 3, 4);
g.add_edge(1, 3, 8);

// Compute the shortest path from vertex 1 to vertex 3
auto sp = graph_algorithms::dijkstra_shortest_path(g, 1, 3);

// Print the shortest path
for (const auto& v : sp) {
 std::cout << v << " ";
}
std::cout << std::endl;

return 0;
}
