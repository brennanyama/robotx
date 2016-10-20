cmake_minimum_required(VERSION 3.3)
project(Mission_Planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp status.h maneuvers.h status.cpp maneuvers.cpp events.h events.cpp shape.h shape.cpp)
add_executable(Mission_Planner ${SOURCE_FILES})