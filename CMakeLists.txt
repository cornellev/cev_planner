# Set the project name for your library
project(cev_planner)

cmake_minimum_required(VERSION 3.22)

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)

find_package(OpenCV REQUIRED)

# Add the include directory
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    PUBLIC ${NLOPT_INCLUDE_DIRS}
    PUBLIC ${OpenCV_INCLUDE_DIRS}
)

# Compile the library (static or shared)
add_library(cev_planner STATIC
    src/global_planner/rrt.cpp
    src/global_planner/rrt_utils.cpp
    src/local_planner/mpc.cpp
    src/cost_map/gaussian_conv.cpp
    src/vis/vis.cpp
    src/cost_map/nearest.cpp
    src/cost_map/nothing.cpp
)

target_link_libraries(cev_planner Eigen3::Eigen ${NLOPT_LIBRARIES} ${OpenCV_LIBS})