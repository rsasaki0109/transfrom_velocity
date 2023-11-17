cmake_minimum_required(VERSION 3.10)
project(transfrom_velocity)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(/usr/include/eigen3)

include_directories(include)
file(GLOB SOURCES "src/*.cpp")

add_executable(transfrom_velocity_main ${SOURCES})