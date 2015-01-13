cmake_minimum_required(VERSION 2.8.4)
project(N_Body)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp)
add_executable(N_Body ${SOURCE_FILES})