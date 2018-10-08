# Copyright (c) 2015-2018 DNEG Visual Effects
#
# All rights reserved. This software is distributed under the
# Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
#
# Redistributions of source code must retain the above copyright
# and license notice and the following restrictions and disclaimer.
#
# *     Neither the name of DNEG Visual Effects nor the names
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

# -*- cmake -*-
# - Find OpenVDB AX
#
# Author : Nick Avramoussis
#
# OpenVDB_AX_FOUND         Set if OpenVDB AX is found.
# OpenVDB_AX_INCLUDE_DIR   OpenVDB AX's include directory
# OpenVDB_AX_LIBRARY_DIR   OpenVDB AX's library directory
# OpenVDB_AX_LIBRARY       OpenVDB AX's library
#

FIND_PACKAGE( PackageHandleStandardArgs )

FIND_PATH( OPENVDB_AX_LOCATION include/openvdb_ax/
  "$ENV{OPENVDB_AX_ROOT}"
  NO_DEFAULT_PATH
  NO_SYSTEM_ENVIRONMENT_PATH
  )

FIND_PACKAGE_HANDLE_STANDARD_ARGS( OpenVDB_AX
  REQUIRED_VARS OPENVDB_AX_LOCATION
  )

IF( OpenVDB_AX_FOUND )
  SET( OpenVDB_AX_INCLUDE_DIR ${OPENVDB_AX_LOCATION}/include
    CACHE PATH "OpenVDB include directory")

  SET( OpenVDB_AX_LIBRARY_DIR ${OPENVDB_AX_LOCATION}/lib
    CACHE PATH "OpenVDB library directory" )

  FIND_LIBRARY( OpenVDB_AX_LIBRARY openvdb_ax
    PATHS ${OpenVDB_AX_LIBRARY_DIR}
    NO_DEFAULT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    )

ENDIF( OpenVDB_AX_FOUND )