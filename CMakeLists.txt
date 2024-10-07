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
  URL https://github.com/pybind/pybind11/archive/refs/tags/v2.13.6.tar.gz
  URL_HASH SHA256=e08cb87f4773da97fa7b5f035de8763abc656d87d5773e62f6da0587d1f0ec20
)
FetchContent_MakeAvailable(pybind11)

set(python_module_name _robosim)

file(GLOB SRC_FILES src/robosim/*.cpp)
file(GLOB HEADER_FILES src/robosim/*.h)
file(GLOB P_SRC_FILES src/robosim/physics/*.cpp)
file(GLOB P_HEADER_FILES src/robosim/physics/*.h)

pybind11_add_module(${python_module_name} MODULE
        ${SRC_FILES}
        ${HEADER_FILES}
        ${P_SRC_FILES}
        ${P_HEADER_FILES}
  )

find_package(ODE REQUIRED)
include_directories(${ODE_INCLUDE_DIRS})


#target_link_libraries(${python_module_name} PUBLIC MyLib)
target_link_libraries(${python_module_name} PUBLIC  ${ODE_LIBRARIES})

install(TARGETS ${python_module_name} DESTINATION .)

# Quiet a warning, since this project is only valid with SKBUILD
set(ignoreMe "${SKBUILD}")



