cmake_minimum_required(VERSION 3.9)
project(quadtree)
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -g -pg")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} src)

include_directories( ${includeDirs} )

set(SRC ${SRC} 
    # src/quadtree_fast.cpp
    src/quadtree/quadtree_array.cpp
    src/quadtree/quadtree_array2.cpp
    src/quadtree/quadtree_array3.cpp
    src/quadtree/quadtree_pointer.cpp
    src/quadtree/quadtree_pointer2.cpp
    src/quadtree/quadtree_hash.cpp
    src/quadtree/quadtree_hash2.cpp
    src/quadtree/quadtree_pointer_refactor.cpp
    src/timer.cpp
)

# add_executable(test_max_elem        ${SRC} src/test_max_elem.cpp)
# add_executable(test_moving_matching ${SRC} src/test_moving_matching.cpp)
add_executable(test_max_depth_array       ${SRC} src/test_max_depth_array.cpp)
add_executable(test_max_depth_array2      ${SRC} src/test_max_depth_array2.cpp)
add_executable(test_max_depth_array3      ${SRC} src/test_max_depth_array3.cpp)
add_executable(test_max_depth_pointer     ${SRC} src/test_max_depth_pointer.cpp)
add_executable(test_max_depth_pointer2    ${SRC} src/test_max_depth_pointer2.cpp)
add_executable(test_max_depth_hash        ${SRC} src/test_max_depth_hash.cpp)
add_executable(test_max_depth_hash2        ${SRC} src/test_max_depth_hash2.cpp)
add_executable(test_max_depth_pointer_refactor ${SRC} src/test_max_depth_pointer_refactor.cpp)


add_executable(test_hash_based     ${SRC} src/test_hash_based.cpp)

add_executable(test_bitcheck     ${SRC} src/test_bitcheck.cpp)
