cmake_minimum_required(VERSION 3.0)
project(curve_fit)

set (CMAKE_CXX_STANDARD 14)
# add_subdirectory(pybind11)
#pybind11_add_module(ceres_python src/bindings.cpp)
#target_link_libraries(ceres_python PRIVATE ceres)

find_package(ceres REQUIRED)
find_package(eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_executable(curve_fit curve_fit.cc)
target_link_libraries(curve_fit ceres)
