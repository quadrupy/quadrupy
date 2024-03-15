cmake_minimum_required(VERSION 2.8.3)
project(inekf_python_wrapper)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DEIGEN_NO_DEBUG -march=native")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# python
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib) # .so .dylib

include(FetchContent)
set(FETCHCONTENT_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/_deps)
set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG        v2.11.1
)
FetchContent_MakeAvailable(pybind11)

FetchContent_Declare(
        inekf
        GIT_REPOSITORY https://github.com/akoen/invariant-ekf
        GIT_TAG master
)
FetchContent_MakeAvailable(inekf)
target_include_directories(inekf PUBLIC ${inekf_SOURCE_DIR}/include)

FetchContent_Declare(
        unitree_interface
        GIT_REPOSITORY https://github.com/akoen/unitree_sdk2
        GIT_TAG main
)
FetchContent_MakeAvailable(unitree_interface)

pybind11_add_module(inekf_py inekf_binding.cpp)
target_link_libraries(inekf_py PRIVATE inekf)
pybind11_add_module(go2_py hardware_binding.cpp)
target_link_libraries(go2_py PRIVATE unitree_interface)