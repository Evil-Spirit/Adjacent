cmake_minimum_required(VERSION 3.1)
project(adjacent)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_PYTHON_BINDINGS "Build Python bindings" ON)

# set(PYTHON_EXECUTABLE /home/wolfv/miniconda3/bin/python)
# set(PYTHON_LIBRARIES /home/wolfv/miniconda3/lib/)

find_package(xtl REQUIRED)
find_package(xtensor REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(include)
include_directories(${xtl_INCLUDE_DIRS})
include_directories(${xtensor_INCLUDE_DIRS})

add_library(adjacent_lib
	src/expression.cpp
	src/expression_vector.cpp
	src/gaussian_method.cpp
	src/equation_system.cpp
	src/expr_basis.cpp
)

target_link_libraries(adjacent_lib)

add_executable(adjacent_test
	src/test.cpp
)

target_link_libraries(adjacent_test adjacent_lib)

if (BUILD_PYTHON_BINDINGS)
	pybind11_add_module(adjacent_api
	    src/py_interface.cpp
	)
	target_link_libraries(adjacent_api PUBLIC pybind11::pybind11 adjacent_lib)
endif()
