############################################################################
#
# Copyright (c) 2020-2021 Auterion AG. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name Auterion nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)

project(configuration-manager)

if(NOT MSVC)
  add_definitions("-Wall -Werror -Wextra -Wno-deprecated-copy -Wno-deprecated-declarations -Wno-maybe-uninitialized -Wno-array-bounds")
else()
  add_definitions("-WX -W2")
endif()

find_package(MAVSDK REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Find DBUS using PkgConfig
find_package(PkgConfig REQUIRED)
pkg_check_modules(DBUS dbus-1)
set(DBUS_LIBS_ABSOLUTE)
foreach(lib ${DBUS_LIBRARIES})
  set(find_result DBUS_${lib}_ABS)
  find_library(${find_result} ${lib} ${DBUS_LIBRARY_DIR})
  list(APPEND DBUS_LIBS_ABSOLUTE ${${find_result}})
endforeach()

include_directories(include)

add_executable(configuration-manager
  main.cpp
  configuration_manager.cpp
  dbus_interface.cpp
  software_interface.cpp
  # Software Interfaces
  software_interfaces/autopilot_manager_config.cpp
)

target_include_directories(configuration-manager
  PRIVATE
  ${DBUS_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../include
)

target_link_libraries(configuration-manager
  MAVSDK::mavsdk_mavlink_passthrough
  MAVSDK::mavsdk
  ${DBUS_LIBS_ABSOLUTE}
  Threads::Threads
)

install(TARGETS configuration-manager
  RUNTIME DESTINATION bin
)
