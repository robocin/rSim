cmake_minimum_required(VERSION 3.14)

project(robosim)
set(CMAKE_CXX_STANDARD 11)

# Define CMAKE_INSTALL_xxx: LIBDIR, INCLUDEDIR
include(GNUInstallDirs)
include(FindPackageHandleStandardArgs)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
# Fetch pybind11
include(FetchContent)

FetchContent_Declare(
  pybind11
  URL https://github.com/pybind/pybind11/archive/refs/tags/v2.6.2.tar.gz
  URL_HASH SHA256=8ff2fff22df038f5cd02cea8af56622bc67f5b64534f1b83b9f133b8366acff2
)
FetchContent_MakeAvailable(pybind11)

set(python_module_name robosim)

#file(GLOB SRC_FILES src/robosim/*.cpp)
#file(GLOB HEADER_FILES src/robosim/*.h)

pybind11_add_module(${python_module_name} MODULE
    src/robosim/robosim_py.cpp
    src/robosim/physics/pworld.cpp
    src/robosim/physics/pobject.cpp
    src/robosim/physics/pball.cpp
    src/robosim/physics/pground.cpp
    src/robosim/physics/pfixedbox.cpp
    src/robosim/physics/pcylinder.cpp
    src/robosim/physics/pbox.cpp
    src/robosim/sslconfig.cpp
    src/robosim/vssconfig.cpp
    src/robosim/vssrobot.cpp
    src/robosim/sslrobot.cpp
    src/robosim/utils.cpp
    src/robosim/vssworld.cpp
    src/robosim/physics/pworld.h
    src/robosim/physics/pobject.h
    src/robosim/physics/pball.h
    src/robosim/physics/pground.h
    src/robosim/physics/pfixedbox.h
    src/robosim/physics/pcylinder.h
    src/robosim/physics/pbox.h
    src/robosim/sslconfig.h
    src/robosim/vssconfig.h
    src/robosim/sslrobot.h
    src/robosim/vssrobot.h
    src/robosim/utils.h
    src/robosim/vssworld.h
  )

find_package(Qt5 COMPONENTS Core Network REQUIRED)
find_package(ODE REQUIRED)
include_directories(${ODE_INCLUDE_DIRS})


#target_link_libraries(${python_module_name} PUBLIC MyLib)
target_link_libraries(${python_module_name} PUBLIC Qt5::Core Qt5::Network)
target_link_libraries(${python_module_name} PUBLIC  ${ODE_LIBRARIES})

install(TARGETS ${python_module_name} DESTINATION .)

# Quiet a warning, since this project is only valid with SKBUILD
set(ignoreMe "${SKBUILD}")



