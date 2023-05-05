#Minimum requirement of CMake version : 3.0.0
cmake_minimum_required(VERSION 3.0.0)

#Project name and version number
project(MultiPosition VERSION 0.1.0)

file(GLOB SRC_FILES "*.cpp" "*.h")

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

find_package(Ceres REQUIRED)
target_link_libraries(MultiPosition ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME} Solver OrbitClock)

