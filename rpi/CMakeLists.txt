cmake_minimum_required(VERSION 3.10)
project(vpos LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-O0 -g3 -ggdb3 -fno-omit-frame-pointer -Wall -Wextra")
set(CMAKE_C_FLAGS   "-O0 -g3 -ggdb3 -fno-omit-frame-pointer -Wall -Wextra")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
#set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall -Wextra -Werror")

# -----------------------------
# Find OpenCV
# -----------------------------
find_package(OpenCV REQUIRED core imgproc videoio highgui calib3d aruco)

# -----------------------------
# Source files
# -----------------------------
add_executable(vpos
    src/main.cpp
    src/comm.cpp
    src/serial.cpp
    src/controller.cpp
    src/vodom.cpp
    src/params.cpp
    src/logger.cpp
    src/tcp_server.cpp
    ../shared/host_interface.c
)

# -----------------------------
# Includes
# -----------------------------
target_include_directories(vpos PRIVATE
    src
    ../shared
    ${OpenCV_INCLUDE_DIRS}
)

# -----------------------------
# Libraries
# -----------------------------
target_link_libraries(vpos
    ${OpenCV_LIBS}
    atomic
)

message(STATUS "OpenCV includes: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")
