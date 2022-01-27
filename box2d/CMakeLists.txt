cmake_minimum_required(VERSION 3.17)
project(box2djai)

set(CMAKE_CXX_STANDARD 20)

#set(BOX2D_INSTALL false)
#set(BOX2D_BUILD_STATIC false)
#set(BOX2D_VERSION 1.1.0)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

message("BUILD_SHARED_LIBS1 ${BUILD_SHARED_LIBS}")

#option(box2d_EXPORTS "Export lib I guess" ON)

add_subdirectory(Box2D)

include_directories(Box2D/include wrapper)

add_library(box2djai SHARED lib.cpp)

target_link_libraries(box2djai PUBLIC box2d)