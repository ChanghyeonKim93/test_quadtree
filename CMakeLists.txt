cmake_minimum_required(VERSION 3.5.1)
project(quadtree)
set(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -pg -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )

set(SRC ${SRC} 
    src/quadtree3.cpp
)

add_executable (test_quadtree3 ${SRC} src/test_main_quadtree3.cpp)
# target_link_libraries (test_quadtree ${OpenCV_LIBRARIES})