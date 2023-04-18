# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.16)

# Set the project name and version
project(graph_algorithms VERSION 1.0)

# Add subdirectories for the library source and include directories
add_subdirectory(src)
add_subdirectory(include)

# Add subdirectory for tests
add_subdirectory(tests)

# Configure a header file to pass some of the CMake settings
# to the source code
configure_file(
  "${PROJECT_SOURCE_DIR}/include/graph_algorithms/config.hpp.in"
  "${PROJECT_BINARY_DIR}/include/graph_algorithms/config.hpp"
)

# Add the library target
add_library(graph_algorithms
  $<TARGET_OBJECTS:algorithm>
  $<TARGET_OBJECTS:graph>
  $<TARGET_OBJECTS:shortest_path_algorithm>
  $<TARGET_OBJECTS:minimum_spanning_tree_algorithm>
)

# Add the library include directories
target_include_directories(graph_algorithms PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
)

# Set the library version and installation directory
set_target_properties(graph_algorithms PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
  PUBLIC_HEADER "${PROJECT_SOURCE_DIR}/include/graph_algorithms/algorithm.hpp"
)

# Configure the installation process
include(GNUInstallDirs)
install(TARGETS graph_algorithms
  EXPORT graph_algorithmsConfig
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/graph_algorithms
)

# Install the CMake package configuration files
install(EXPORT graph_algorithmsConfig
  NAMESPACE graph_algorithms::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/graph_algorithms/cmake
)

# Create a package configuration file for the build directory
export(TARGETS graph_algorithms
  FILE "${PROJECT_BINARY_DIR}/graph_algorithmsConfig.cmake"
  NAMESPACE graph_algorithms::
)

# Create a package configuration file for the install directory
install(EXPORT graph_algorithmsConfig
  NAMESPACE graph_algorithms::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/graph_algorithms/cmake
)

# Enable testing for the library
enable_testing()

# Add the test target
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)

# Add the test subdirectories
add_subdirectory(tests)