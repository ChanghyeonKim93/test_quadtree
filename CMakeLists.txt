cmake_minimum_required(VERSION 3.5.1)
project(quadtree)
set(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -O2 -g -pg")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )

set(SRC ${SRC} 
    # src/quadtree_fast.cpp
    src/quadtree_fast2.cpp
    src/timer.cpp
)

add_executable(test_max_elem        ${SRC} src/test_max_elem.cpp)
# add_executable(test_moving_matching ${SRC} src/test_moving_matching.cpp)
# add_executable(test_max_depth       ${SRC} src/test_max_depth.cpp)
# add_executable(test_bit_interleave  ${SRC} src/test_bit_interleave.cpp)
# add_executable(test_bitshift        ${SRC} src/test_bitshift.cpp)
# target_link_libraries (test_quadtree ${OpenCV_LIBRARIES})