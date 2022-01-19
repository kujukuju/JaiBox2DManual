# Install script for directory: C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/box2djai")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Box2D" TYPE FILE FILES "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Box2D.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Box2D/Collision" TYPE FILE FILES
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Collision/b2BroadPhase.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Collision/b2Collision.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Collision/b2Distance.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Collision/b2DynamicTree.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Collision/b2TimeOfImpact.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Box2D/Collision/Shapes" TYPE FILE FILES
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Collision/Shapes/b2CircleShape.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Collision/Shapes/b2EdgeShape.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Collision/Shapes/b2ChainShape.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Collision/Shapes/b2PolygonShape.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Collision/Shapes/b2Shape.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Box2D/Common" TYPE FILE FILES
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Common/b2BlockAllocator.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Common/b2Draw.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Common/b2GrowableStack.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Common/b2Math.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Common/b2Settings.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Common/b2StackAllocator.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Common/b2Timer.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Box2D/Dynamics" TYPE FILE FILES
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/b2Body.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/b2ContactManager.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/b2Fixture.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/b2Island.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/b2TimeStep.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/b2World.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/b2WorldCallbacks.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Box2D/Dynamics/Contacts" TYPE FILE FILES
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Contacts/b2CircleContact.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Contacts/b2Contact.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Contacts/b2ContactSolver.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Contacts/b2PolygonAndCircleContact.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Contacts/b2EdgeAndCircleContact.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Contacts/b2EdgeAndPolygonContact.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Contacts/b2ChainAndCircleContact.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Contacts/b2ChainAndPolygonContact.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Contacts/b2PolygonContact.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Box2D/Dynamics/Joints" TYPE FILE FILES
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Joints/b2DistanceJoint.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Joints/b2FrictionJoint.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Joints/b2GearJoint.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Joints/b2Joint.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Joints/b2MotorJoint.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Joints/b2MouseJoint.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Joints/b2PrismaticJoint.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Joints/b2PulleyJoint.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Joints/b2RevoluteJoint.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Joints/b2RopeJoint.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Joints/b2WeldJoint.h"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Dynamics/Joints/b2WheelJoint.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Box2D/Rope" TYPE FILE FILES "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/Rope/b2Rope.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "C:/Users/Kuju/Projects/JaiBox2D/box2d/cmake-build-relwithdebinfo/Box2D/Box2D/Box2D.lib")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/Box2D/Box2D-targets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}/Box2D/Box2D-targets.cmake"
         "C:/Users/Kuju/Projects/JaiBox2D/box2d/cmake-build-relwithdebinfo/Box2D/Box2D/CMakeFiles/Export/_Box2D/Box2D-targets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}/Box2D/Box2D-targets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}/Box2D/Box2D-targets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Box2D/Box2D-targets.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/Box2D" TYPE FILE FILES "C:/Users/Kuju/Projects/JaiBox2D/box2d/cmake-build-relwithdebinfo/Box2D/Box2D/CMakeFiles/Export/_Box2D/Box2D-targets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/Box2D/Box2D-targets-relwithdebinfo.cmake")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/Box2D" TYPE FILE FILES "C:/Users/Kuju/Projects/JaiBox2D/box2d/cmake-build-relwithdebinfo/Box2D/Box2D/CMakeFiles/Export/_Box2D/Box2D-targets-relwithdebinfo.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/cmake/Box2D/Box2DConfig.cmake;/cmake/Box2D/UseBox2D.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/cmake/Box2D" TYPE FILE FILES
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/cmake-build-relwithdebinfo/Box2D/Box2D/Box2DConfig.cmake"
    "C:/Users/Kuju/Projects/JaiBox2D/box2d/Box2D/Box2D/UseBox2D.cmake"
    )
endif()

