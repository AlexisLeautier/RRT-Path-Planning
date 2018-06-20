cmake_minimum_required(VERSION 2.6)
project(path_planning)

add_executable(path_planning main.cpp RRT.cpp)
install(TARGETS path_planning DESTINATION bin)
