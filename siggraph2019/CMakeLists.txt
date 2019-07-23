# Copyright (c) 2015-2019 DNEG
#
# All rights reserved. This software is distributed under the
# Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
#
# Redistributions of source code must retain the above copyright
# and license notice and the following restrictions and disclaimer.
#
# *     Neither the name of DNEG nor the names of
# its contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# IN NO EVENT SHALL THE COPYRIGHT HOLDERS' AND CONTRIBUTORS' AGGREGATE
# LIABILITY FOR ALL CLAIMS REGARDLESS OF THEIR BASIS EXCEED US$250.00.
#

project(OpenVDBAXSiggraphExamples)
cmake_minimum_required(VERSION 3.3)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake;${CMAKE_CURRENT_LIST_DIR}/../cmake/backports")

# Configure LLVM

find_package(LLVM REQUIRED)
find_library(found_LLVM LLVM HINTS ${LLVM_LIBRARY_DIRS})

if(found_LLVM)
  set(LLVM_LIBS "LLVM")
else()
  llvm_map_components_to_libnames(_llvm_libs all)
  set(LLVM_LIBS "${_llvm_libs}")
endif()

message(STATUS "Found LLVM: ${LLVM_DIR} (found version \"${LLVM_PACKAGE_VERSION}\")")

if(LLVM_VERSION_MAJOR GREATER 5)
  message(WARNING "The version of LLVM located is greater than 5. "
    "OpenVDB AX has been tested fully against LLVM 5.0.0 and 5.0.1, it is "
    "recommended that you use this version where possible. Please report "
    "any issues encountered with newer versions."
  )
endif()

if(USE_HOUDINI)
  include(OpenVDBHoudiniSetup)
endif()
find_package(OpenVDB REQUIRED openvdb openvdb_ax)

##### sg2019

set(SOURCE_FILES main.cc)
add_executable(sg2019 ${SOURCE_FILES})

target_compile_definitions(sg2019 PUBLIC ${LLVM_DEFINITIONS})
target_include_directories(sg2019 SYSTEM PUBLIC ${LLVM_INCLUDE_DIRS})

target_link_libraries(sg2019
  ${LLVM_LIBS}
  OpenVDB::openvdb
  OpenVDB::openvdb_ax
  )
