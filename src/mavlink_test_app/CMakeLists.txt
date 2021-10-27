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
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(mavlink-test-app)

find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)

if(NOT MSVC)
  add_definitions("-Wall -Werror -Wextra -Wno-deprecated-copy -Wno-deprecated-declarations -Wno-maybe-uninitialized -Wno-array-bounds")
else()
  add_definitions("-WX -W2")
endif()

add_executable(${PROJECT_NAME}
  main.cpp
  mavlink_handler.cpp
  mavlink_parameters.cpp
)

include_directories(
  ${PROJECT_SOURCE_DIR}/../mavlink_test_app
)

target_link_libraries(${PROJECT_NAME}
  MAVSDK::mavsdk_mavlink_passthrough
  MAVSDK::mavsdk
  pthread
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
)
