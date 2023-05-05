#Minimum requirement of CMake version : 3.0.0
cmake_minimum_required(VERSION 3.0.0)

#Project name and version number
project(POSGO VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++11 -O0 -fPIC")  #设置c++的编译选项
set(CMAKE_CXX_FLAGS "-msse3")

set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -g")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -ggdb")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

file(GLOB GLOBAL "src/Global.cpp" "src/Global.h" )

add_executable(POSGO app/POSGO.cpp ${GLOBAL})


target_link_libraries(POSGO MultiPosition SensorErrorModel SystemSetting DataManagement ResultExport)

add_subdirectory("src/SystemSetting")
add_subdirectory("src/ResultExport")
add_subdirectory("src/DataManagement")
add_subdirectory("src/MultiPosition")
add_subdirectory("src/OrbitClock")
add_subdirectory("src/SensorErrorModel")
add_subdirectory("src/Solver")

#Choose different compilation configurations according to VS compilation
if(CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-O3)
    set(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build_release)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build_release/Lib)
elseif(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-O0)
    set(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build_debug)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build_debug/Lib)
endif()

add_compile_options(-w)
add_compile_options(-m64)
add_compile_options(-lz)
add_compile_options(-lstdc++)

#Output Messages for debug the Cmake
message(STATUS "operation system is : ${CMAKE_SYSTEM}")
message(STATUS "current platform is : ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMake version is    : ${CMAKE_SYSTEM_VERSION}")
message(STATUS "C compiler is : ${CMAKE_C_COMPILER}")
message(STATUS "C++ compiler is : ${CMAKE_CXX_COMPILER}")
message(STATUS "The program main directory is : ${PROJECT_SOURCE_DIR}")
