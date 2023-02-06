cmake_minimum_required(VERSION 3.5)

project(MCTS_PLANNER)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(third_party/pybind11)

include_directories(mcts_planner/include)
include_directories(third_party/Epsilon-Intersection/include)

add_subdirectory(mcts_planner)