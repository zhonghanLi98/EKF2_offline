############################################################################
#
#   Copyright (c) 2015 ECL Development Team. All rights reserved.
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
# 3. Neither the name ECL nor the names of its contributors may be
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

cmake_minimum_required(VERSION 2.8)

project (ECL CXX)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-DPOSIX_SHARED")

#set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
#message("looking for FindEigen3.cmake in ${CMAKE_MODULE_PATH}")
#find_package(Eigen3 REQUIRED)

#if( NOT EIGEN3_INCLUDE_DIR )
#	message( FATAL_ERROR "Eigen3 not found.")
#endif()

# 检查matrix目录是否存在
if( NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../matrix" )
message( FATAL_ERROR "The matrix library is not available. Please ensure the matrix directory exists."
)
endif()

include_directories(
	./
	../
	../matrix
	#${EIGEN3_INCLUDE_DIR}
)
set(SRCS
	estimator_interface.cpp
	ekf.cpp
	ekf_helper.cpp
	covariance.cpp
	vel_pos_fusion.cpp
	mag_fusion.cpp
	gps_checks.cpp
	airspeed_fusion.cpp
	sideslip_fusion.cpp
	control.cpp
	geo.cpp
	mathlib.cpp
)
#add_definitions(-std=c++11 -Wall -Werror)
add_definitions(-std=c++11  )
# Eigen throws various warnings
#add_definitions(-Wno-deprecated-declarations -Wno-enum-compare -Wno-unused-local-typedefs)

add_library(ecl STATIC ${SRCS})

#add_library(myekf2 SHARED Myekf2.cpp)
#target_link_libraries(myekf2 ecl  )

add_executable(ekf_offline Myekf2.cpp)
target_link_libraries(ekf_offline ecl  )
 
#add_executable(main main.cpp)
#target_link_libraries(main ecl  myekf2)
 



