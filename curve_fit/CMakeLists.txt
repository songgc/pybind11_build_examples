cmake_minimum_required(VERSION 3.0)
project(curve_fit)

set (CMAKE_CXX_STANDARD 14)
find_package(pybind11 REQUIRED)
find_package(ceres REQUIRED)
find_package(eigen3 REQUIRED)
find_package(Sophus REQUIRED)
# find_package(python)
include_directories(${EIGEN_INCLUDE_DIRS})

pybind11_add_module(curve_fit src/curve_fit.cc)
pybind11_add_module(pose_estimation src/pose_estimation.cc)
target_link_libraries(curve_fit PRIVATE ceres)
target_link_libraries(pose_estimation PRIVATE ceres)



