cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(exercise_01)

# Options

find_package (Eigen3 3.3 REQUIRED)
find_library(FREEIMAGE_LIB NAMES freeimage libfreeimage FreeImage libFreeImage PATH_SUFFIXES dynamic)

# Set C++ flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(HEADERS
    Eigen.h
    FreeImageHelper.h
    VirtualSensor.h
)

set(SOURCES
    main.cpp
    FreeImageHelper.cpp
)

add_executable(exercise_01 ${HEADERS} ${SOURCES})

target_link_libraries(exercise_01 Eigen3::Eigen freeimage)
