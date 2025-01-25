# Set the project name for your library
project(cev_planner)

cmake_minimum_required(VERSION 3.22)

# Add the include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Compile the library (static or shared)
add_library(cev_planner STATIC
)
