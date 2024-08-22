cmake_minimum_required(VERSION 3.10)
project(KalmanFilter)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(EIGEN3_INCLUDE_DIR "C:/MagnusProjekter/KalmanFilter/libs/eigen")
include_directories(${EIGEN3_INCLUDE_DIR})


# Specify the include directories
include_directories(include)

# Add source files
set(SOURCES
    src/main.cpp
    src/KalmanFilter.cpp
    src/Simulate.cpp
)

# Create the executable
add_executable(KalmanFilter ${SOURCES})
