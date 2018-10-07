cmake_minimum_required(VERSION 2.8)
project(optimization_GPSIMU)

#[[IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()]]
set(CMAKE_BUILD_TYPE Debug)
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#[[set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")]]
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0  -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O0  -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Eigen3 3.1.0 REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
)

#for mapmatcher ;
set(optimization_SOURCE_FILES
        include/paramReader.h
        src/paramReader.cpp
        src/main.cpp
        include/struct.h src/factors.cpp src/factors.h src/math_utility.cpp src/math_utility.h)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


# Build examples

add_executable(optimization_GPSIMU ${optimization_SOURCE_FILES})
target_link_libraries(optimization_GPSIMU ${CERES_LIBRARIES})
