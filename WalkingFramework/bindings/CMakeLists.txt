cmake_minimum_required(VERSION 2.8.3)
project(inekf_python_wrapper)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/inekf/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# python
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib) # .so .dylib

add_subdirectory(../third_party/pybind ../third_party/pybind/build)

add_subdirectory(../third_party/inekf ../third_party/inekf/build)

pybind11_add_module(inekf_py inekf_binding.cpp)

target_link_libraries(inekf_py PRIVATE inekf)

