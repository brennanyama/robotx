cmake_minimum_required(VERSION 3.3)
project(Mission_Planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp status.h maneuvers.h status.cpp maneuvers.cpp)
add_executable(Mission_Planner ${SOURCE_FILES})