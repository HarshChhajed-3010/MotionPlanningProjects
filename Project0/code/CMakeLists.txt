cmake_minimum_required(VERSION 3.10.1)
project(Project0)

set(CMAKE_CXX_STANDARD 11)

find_package(ompl REQUIRED)

add_executable(Point2DPlanning Point2DPlanning.cpp)
add_executable(RigidBodyPlanning RigidBodyPlanning.cpp)
add_executable(PlannerData PlannerData.cpp)

#If you install from master you will have to link the new way by uncommentting the following line and commenting the rest.
#target_link_libraries(Point2DPlanning ompl::ompl)
target_link_libraries(Point2DPlanning PRIVATE ${OMPL_LIBRARIES})
target_include_directories(Point2DPlanning PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_directories(Point2DPlanning PRIVATE ${OMPL_INCLUDE_DIRS})

target_link_libraries(RigidBodyPlanning PRIVATE ${OMPL_LIBRARIES})
target_include_directories(RigidBodyPlanning PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_directories(RigidBodyPlanning PRIVATE ${OMPL_INCLUDE_DIRS})

target_link_libraries(PlannerData PRIVATE ${OMPL_LIBRARIES})
target_include_directories(PlannerData PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_directories(PlannerData PRIVATE ${OMPL_INCLUDE_DIRS})