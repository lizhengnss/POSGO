#Minimum requirement of CMake version : 3.0.0
cmake_minimum_required(VERSION 3.0.0)

#Project name and version number
project(ResultExport VERSION 0.1.0)

file(GLOB SRC_FILES "*.cpp" "*.h")

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})



