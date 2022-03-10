cmake_minimum_required(VERSION 3.0)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

project(path_planner_2d)

# Algorithms

# 1) A* Star
add_executable(AStar "A*Star Algorithm/test/main.cpp")

# 2) A* Star Bidirectional
add_executable(BiAStar "A*Star Bidirectional/test/main.cpp")

# 3) Dijkstra Algorithm
add_executable(Dijkstra "Dijkstra Algorithm/test/main.cpp")

# 4) Dijkstra Bidirectional
add_executable(BiDijkstra "Dijkstra Bidirectional/test/main.cpp")

# 5) Greedy Best First Search
add_executable(GreedyBFS "Greedy Best First Search/test/main.cpp")