cmake_minimum_required(VERSION 3.5)


## Project branding, version and package mantainer
project(RoboSim)
set(VERSION "1.0.0")
set(MAINTAINER "Mateus Machado, Felipe Martins, Tiago Barros <(mgm4, fbm2, tsb)@cin.ufpe.br> ")

# some utils and helper vars
string(TOLOWER ${CMAKE_PROJECT_NAME} CMAKE_PROJECT_NAME_LOWER)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
include(${PROJECT_SOURCE_DIR}/cmake/Utils.cmake)
standard_config()
standard_paths(${PROJECT_SOURCE_DIR} bin lib)

set(app ${CMAKE_PROJECT_NAME})
# create the target before the sources list is known so that we can call
# add_dependencies(<target> external_proj)
add_executable(${app} "" src/simulator.cpp)

# definitions for knowing the OS from the code
if(MSVC)
    add_definitions(-DHAVE_MSVC)
endif()
if(WIN32)
    add_definitions(-DHAVE_WINDOWS)
endif()
if(UNIX)
    add_definitions(-DHAVE_UNIX)
    if(APPLE)
        add_definitions(-DHAVE_MACOSX)
    else()
        #TODO: fix this, say we have FreeBSD, that's not linux
        add_definitions(-DHAVE_LINUX)
    endif()
endif()

# set explicitly the c++ standard to use
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread")

# add src dir to included directories
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/physics)


## Handling depenendcies

# we will append all libs to this var
set(libs)

# OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
list(APPEND libs ${OPENGL_LIBRARIES})
#find_package(GLUT REQUIRED)
#include_directories(${GLUT_INCLUDE_DIR})

# Qt
if(APPLE AND EXISTS /usr/local/opt/qt)
  # Homebrew installs Qt5 (up to at least 5.9.1) in
  # /usr/local/qt5, ensure it can be found by CMake since
  # it is not in the default /usr/local prefix.
  list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/qt")
endif()
find_package(Qt5 COMPONENTS Core Network REQUIRED)
list(APPEND libs Qt5::Core Qt5::Network)

# ODE
find_package(ODE REQUIRED)
include_directories(${ODE_INCLUDE_DIRS})
list(APPEND libs ${ODE_LIBRARIES})
#TODO: make this an option
option(DOUBLE_PRECISION "Use double precision? If not single precision will be used." ON)
  if(DOUBLE_PRECISION)
    if(APPLE)
      add_definitions(-DdIDEDOUBLE)
    else()
      add_definitions(-DdDOUBLE)
    endif()
  else()
    if(APPLE)
      add_definitions(-DdIDESINGLE)
    else()
      add_definitions(-DdSINGLE)
    endif()
  endif()

set(SOURCES
    src/physics/pworld.cpp
    src/physics/pobject.cpp
    src/physics/pball.cpp
    src/physics/pground.cpp
    src/physics/pfixedbox.cpp
    src/physics/pcylinder.cpp
    src/physics/pbox.cpp
    src/speed_estimator.cpp
    src/config.cpp
    src/robot.cpp
    src/world.cpp
    src/simulator.cpp
)

set(HEADERS
    src/physics/pworld.h
    src/physics/pobject.h
    src/physics/pball.h
    src/physics/pground.h
    src/physics/pfixedbox.h
    src/physics/pcylinder.h
    src/physics/pbox.h
    src/speed_estimator.h
    src/config.h
    src/robot.h
    src/world.h
    src/simulator.h
)

# files to be compiled
set(srcs
    ${HEADERS}
    ${SOURCES}
)

target_sources(${app} PRIVATE ${srcs})
install(TARGETS ${app} DESTINATION bin)
target_link_libraries(${app} ${libs})
