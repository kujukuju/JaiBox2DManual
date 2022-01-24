cmake_minimum_required(VERSION 3.17)
project(assertgen)

set(CMAKE_CXX_STANDARD 20)

# set(BOX2D_INSTALL true)
# set(BOX2D_BUILD_STATIC true)
# set(BOX2D_VERSION 1.1.0)

set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_subdirectory(../box2d/Box2D "${CMAKE_BINARY_DIR}/box2d")

message("${CMAKE_BINARY_DIR}/box2d/bin")
message("${CMAKE_BINARY_DIR}")
file(COPY "${CMAKE_BINARY_DIR}/box2d/bin/Debug/box2d.dll" DESTINATION "${CMAKE_BINARY_DIR}/Debug")

include_directories(../box2d/Box2D/include ../box2d/wrapper)

add_executable(assertgen main.cpp)

#message(${CMAKE_SOURCE_DIR}/box2d/bin/Debug)
#find_library(OTHERDLL NAMES box2d PATHS ${CMAKE_SOURCE_DIR}/box2d/bin/Debug)
#message(STATUS "find_library returned ${OTHERDLL}")
#add_library(box2dtest SHARED IMPORTED)
#set_property(TARGET box2dtest PROPERTY IMPORTED_LOCATION "${OTHERDLL}")

target_link_libraries(assertgen PUBLIC box2d)
