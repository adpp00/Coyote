######################################################################################
# This file is part of the Coyote <https://github.com/fpgasystems/Coyote>
# 
# MIT Licence
# Copyright (c) 2025, Systems Group, ETH Zurich
# All rights reserved.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
######################################################################################

# Helper script, determines whether Vitis HLS is available in the system
cmake_minimum_required(VERSION 3.5)

find_path(VITIS_HLS_PATH
  NAMES vitis_hls
  PATHS ${VITIS_HLS_ROOT_DIR} ENV XILINX_VITIS_HLS ENV XILINX_HLS ENV VITIS_HLS
  PATH_SUFFIXES bin
)

if(NOT EXISTS ${VITIS_HLS_PATH})
  message(WARNING "Vitis HLS not found. Please install it before building Coyote.")
else()
  get_filename_component(VITIS_HLS_ROOT_DIR ${VITIS_HLS_PATH} DIRECTORY)
  set(VITIS_HLS 1)
  set(VITIS_HLS_FOUND TRUE)
  set(VITIS_HLS_BINARY ${VITIS_HLS_ROOT_DIR}/bin/vitis_hls)
  set(VITIS_HLS_INCLUDE_DIRS ${VITIS_HLS_ROOT_DIR}/include/)
  message(STATUS "Found Vitis HLS at ${VITIS_HLS_ROOT_DIR}.")
endif()
