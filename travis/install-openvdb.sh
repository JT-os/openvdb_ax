#!/bin/bash -e
#
# Copyright (c) 2015-2019 DNEG
#
# All rights reserved. This software is distributed under the
# Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
#
# Redistributions of source code must retain the above copyright
# and license notice and the following restrictions and disclaimer.
#
# *     Neither the name of DNEG nor the names
# of its contributors may be used to endorse or promote products derived
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
# Builds OpenVDB
#
# Author: Matt Warner

echo "Building and installing openvdb..."

export BLOSC_ROOT=/usr
export BOOST_ROOT=/usr
export GLFW_ROOT=/usr
export ILMBASE_ROOT=/usr
export OPENEXR_ROOT=/usr
export TBB_ROOT=/usr

git clone https://github.com/AcademySoftwareFoundation/openvdb.git openvdb
mkdir openvdb/build
cd openvdb/build

cmake \
    -D CMAKE_CXX_COMPILER=g++ \
    -D CMAKE_C_COMPILER=gcc \
    -D OPENVDB_ABI_VERSION_NUMBER=4 \
    -D USE_GLFW3=OFF \
    -D OPENVDB_BUILD_CORE=ON \
    -D OPENVDB_BUILD_BINARIES=OFF \
    -D OPENVDB_BUILD_PYTHON_MODULE=OFF \
    -D OPENVDB_BUILD_UNITTESTS=OFF \
    -D OPENVDB_BUILD_DOCS=OFF \
    -D OPENVDB_BUILD_HOUDINI_PLUGIN=OFF \
    -D OPENVDB_BUILD_MAYA_PLUGIN=OFF \
    -D CMAKE_INSTALL_PREFIX=/usr \
    ../

sudo make -j2

sudo make install -j2
