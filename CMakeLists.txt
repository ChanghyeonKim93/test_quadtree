cmake_minimum_required(VERSION 2.8.3)
project(quadtree)
#find_package(cmake_modules)
add_compile_options(-std=c++11)
set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

#set(SRC ${SRC} main.cpp kdtree_kch.h kdtree_kch_class.h)
set(SRC ${SRC} main.cpp Quadtree.h)

include_directories( ${includeDirs} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -O2 -m64 -Ofast -flto -msse2 -msse3 -march=native -mfpmath=sse")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -g3 -m64 -Ofast -flto -msse2 -msse3 -march=native -mfpmath=sse")

add_executable (quadtree ${SRC} )
