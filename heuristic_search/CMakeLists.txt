cmake_minimum_required(VERSION 3.6)
project(heuristic_search)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES graph.cpp env.cpp astar.cpp)
add_library(heuristic_search ${SOURCE_FILES})
