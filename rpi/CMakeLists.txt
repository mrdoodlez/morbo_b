cmake_minimum_required(VERSION 3.10)
project(vpos LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(vpos
    src/main.cpp
    src/comm.cpp
    src/serial.cpp
    ../shared/host_interface.c
)

target_include_directories(vpos PRIVATE
    src
    ../shared
)
