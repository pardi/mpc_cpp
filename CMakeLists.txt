cmake_minimum_required(VERSION 3.10)
project(mpc_cpp)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel.")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    # Set compiler flags for optimization
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
endif()

# Collect files
file(GLOB HEADER_FILES include/*.h*)
file(GLOB SRC_FILES src/*.cpp)

# Simple testing main
add_executable(${PROJECT_NAME} ${SRC_FILES})

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
target_link_libraries(${PROJECT_NAME} PRIVATE)