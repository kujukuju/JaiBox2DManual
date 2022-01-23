cmake_minimum_required(VERSION 3.17)
project(assertgen)

set(CMAKE_CXX_STANDARD 20)

# set(BOX2D_INSTALL true)
# set(BOX2D_BUILD_STATIC true)
# set(BOX2D_VERSION 1.1.0)

add_subdirectory(../box2d/Box2D ../box2d)

include_directories(../box2d/Box2D/include ../box2d/wrapper)

add_executable(assertgen main.cpp)

target_link_libraries(assertgen PUBLIC box2d)
