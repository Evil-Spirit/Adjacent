cmake_minimum_required(VERSION 3.1)
project(adjacent)

set(CMAKE_CXX_FLAGS "-Og -std=c++14")

include_directories("./include")

add_executable(adjacent 
	src/expression.cpp
	src/expression_vector.cpp
	src/gaussian_method.cpp
	src/equation_system.cpp
	src/expr_basis.cpp

	src/test.cpp)