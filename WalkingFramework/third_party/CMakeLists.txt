cmake_minimum_required(VERSION 2.8.3)
project(inekf_python_wrapper)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inekf/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# python
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib) # .so .dylib

add_subdirectory(pybind)

add_subdirectory(inekf)

pybind11_add_module(inekf_py inekf_binding.cpp)

target_link_libraries(inekf_py inekf)

