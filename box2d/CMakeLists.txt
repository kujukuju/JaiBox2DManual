cmake_minimum_required(VERSION 3.17)
project(box2djai)

set(CMAKE_CXX_STANDARD 20)

set(BOX2D_INSTALL true)
set(BOX2D_BUILD_STATIC true)
set(BOX2D_VERSION 1.1.0)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_subdirectory(Box2D/Box2D)

include_directories(Box2D wrapper)

add_library(box2djai SHARED lib.cpp)

target_link_libraries(box2djai PUBLIC Box2D)