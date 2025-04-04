cmake_minimum_required(VERSION 3.15)
project(efficient_ransac LANGUAGES CXX)

if (CMAKE_VERSION VERSION_LESS 3.18)
  set(DEV_MODULE Development)
else()
  set(DEV_MODULE Development.Module)
endif()

find_package(Python 3.10 COMPONENTS Interpreter ${DEV_MODULE} REQUIRED)

# set(CMAKE_BUILD_TYPE Debug)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
# pcl
find_package(PCL 1.4 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# yaml-cpp
find_package(yaml-cpp REQUIRED)

# detect the installed nanobind package and import it into CMake
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

# nanobind module
nanobind_add_module(
  _efficient_ransac_mod
  efficient_ransac/bindings.cpp
  efficient_ransac/viewer/viewer.cpp
  efficient_ransac/detector/detector.cpp
  efficient_ransac/shapes/shape.cpp
  efficient_ransac/shapes/plane.cpp
  efficient_ransac/shapes/cylinder.cpp
  efficient_ransac/shapes/sphere.cpp
  efficient_ransac/shapes/bitmap.cpp
)

target_link_libraries(_efficient_ransac_mod PRIVATE ${PCL_LIBRARIES} yaml-cpp)

install(TARGETS _efficient_ransac_mod DESTINATION efficient_ransac)
