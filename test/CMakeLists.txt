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

project(mav-settings-test)

if(NOT MSVC)
  add_definitions("-Wall -Werror -Wextra -Wno-deprecated-copy -Wno-deprecated-declarations -Wno-maybe-uninitialized -Wno-array-bounds")
else()
  add_definitions("-WX -W2")
endif()

find_package(MAVSDK REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(${GTEST_INCLUDE_DIR})

add_executable(mav_settings_test
  mav_settings_test.cpp
)

get_directory_property(PARENT_PROJECT_DIR PARENT_DIRECTORY)
target_include_directories(mav_settings_test
  PRIVATE
  ${PARENT_PROJECT_DIR}/include
)

target_link_libraries(mav_settings_test
  MAVSDK::mavsdk_mavlink_passthrough
  MAVSDK::mavsdk
  Threads::Threads
  ${GTEST_BOTH_LIBRARIES}
)

ADD_TEST(mav_settings_test_build
  "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target mav_settings_test
)
ADD_TEST(mav_settings_test mav_settings_test)
SET_TESTS_PROPERTIES(mav_settings_test
  PROPERTIES DEPENDS mav_settings_test_build
)
