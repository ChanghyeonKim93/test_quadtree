cmake_minimum_required(VERSION 3.9)
project(quadtree)
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -g -pg")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )

set(SRC ${SRC} 
    # src/quadtree_fast.cpp
    src/quadtree/quadtree_array.cpp
    src/quadtree/quadtree_array2.cpp
    src/quadtree/quadtree_pointer.cpp
    src/quadtree/quadtree_pointer2.cpp
    src/timer.cpp
)

# add_executable(test_max_elem        ${SRC} src/test_max_elem.cpp)
# add_executable(test_moving_matching ${SRC} src/test_moving_matching.cpp)
add_executable(test_max_depth_array       ${SRC} src/test_max_depth_array.cpp)
add_executable(test_max_depth_array2      ${SRC} src/test_max_depth_array2.cpp)
add_executable(test_max_depth_pointer     ${SRC} src/test_max_depth_pointer.cpp)
add_executable(test_max_depth_pointer2     ${SRC} src/test_max_depth_pointer2.cpp)