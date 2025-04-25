cmake_minimum_required(VERSION 3.10)
project(Project4)

set(CMAKE_CXX_STANDARD 11)
find_package(ompl REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

# Car executable
add_executable(car car.cpp CollisionChecking.cpp)
target_link_libraries(car ${OMPL_LIBRARIES})

# Pendulum executable with RG-RRT
add_executable(pendulum 
    pendulum.cpp
    RG-RRT.cpp  # Only include RG-RRT files here
    RG-RRT.h
)
target_link_libraries(pendulum ${OMPL_LIBRARIES})