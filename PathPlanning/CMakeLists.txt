cmake_minimum_required(VERSION 2.6)
set(CMAKE_CXX_STANDARD 11)

project(path_planning)

add_executable(path_planning main.cpp rrt.cpp node.cpp obstacle.cpp)
install(TARGETS path_planning DESTINATION bin)
