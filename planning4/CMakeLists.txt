cmake_minimum_required(VERSION 3.22)
project(planning_cli LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(casadi REQUIRED)

add_library(planning_lib STATIC
    src/trajectory_optimizer.cpp
    src/exporter.cpp
)

target_include_directories(planning_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(planning_lib PUBLIC casadi)

add_executable(planning_cli src/main.cpp)
target_link_libraries(planning_cli planning_lib)
