# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.22.1)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(pybind11-populate NONE)



include(ExternalProject)
ExternalProject_Add(pybind11-populate
                     "UPDATE_DISCONNECTED" "False" "URL" "https://github.com/pybind/pybind11/archive/refs/tags/v2.6.2.tar.gz" "URL_HASH" "SHA256=8ff2fff22df038f5cd02cea8af56622bc67f5b64534f1b83b9f133b8366acff2"
                    SOURCE_DIR          "/home/rc-blackout/rSim/build/_deps/pybind11-src"
                    BINARY_DIR          "/home/rc-blackout/rSim/build/_deps/pybind11-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)


