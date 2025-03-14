cmake_minimum_required(VERSION 3.10)

project(Project2)

set(CMAKE_CXX_STANDARD 11)

find_package(ompl REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

# Create a library from CollisionChecking.cpp and RTP.cpp
add_library(MY_LIBRARY CollisionChecking.cpp RTP.cpp)
target_link_libraries(MY_LIBRARY ${OMPL_LIBRARIES})

# Executable for motion planning
add_executable(planRTP PlanningRTP.cpp)
target_link_libraries(planRTP MY_LIBRARY ${OMPL_LIBRARIES})

# ✅ New: Executable for Kinematic Chain Benchmark
add_executable(KinematicChainBenchmark KinematicChainBenchmark.cpp)
target_link_libraries(KinematicChainBenchmark MY_LIBRARY ${OMPL_LIBRARIES})
