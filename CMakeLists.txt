cmake_minimum_required(VERSION 3.5)
project(etool_bms CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

add_subdirectory(SocketCan)
add_subdirectory(examples)

add_executable(etool
        examples/can_example.cpp)



