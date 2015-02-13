################################################################################
#
# CMake file for hostif library where target is Altera ARM
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# Copyright (c) 2015, Kalycito Infotech Private Limited
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holders nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

################################################################################
# Handle includes
SET(CMAKE_MODULE_PATH "${OPLK_BASE_DIR}/cmake" ${CMAKE_MODULE_PATH})
# Include target specific global configuration file
INCLUDE(setalteraarmboardconfig)

################################################################################
# Add support for Altera Designs in hostinterface
# Set architecture specific sources and include directories

SET(LIB_ARCH_SRCS
        ${PROJECT_SOURCE_DIR}/src/hostiflibint-altera_arm.c
    )

SET(LIB_ARCH_INCS
                    ${EXAMPLE_BINARY_DIR}/bsp${CFG_${PROC_INST_NAME}_NAME}/${CFG_${PROC_INST_NAME}_NAME}/include
                   )

################################################################################
# Set architecture specific definitions
ADD_DEFINITIONS(${ALT_${PROC_INST_NAME}_CFLAGS} -D__altera_arm__ )

################################################################################
# Set architecture specific installation files
################################################################################
# Eclipse project files
SET(CFG_CPU_NAME ${CFG_${PROC_INST_NAME}_NAME})

GEN_ECLIPSE_FILE_LIST("${HOSTIF_LIB_SRCS}" "" PART_ECLIPSE_FILE_LIST)
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")
GEN_ECLIPSE_FILE_LIST("${LIB_ARCH_SRCS}" "arch" PART_ECLIPSE_FILE_LIST)
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")
GEN_ECLIPSE_INCLUDE_LIST("${HOSTIF_LIB_INCS}" PART_ECLIPSE_INCLUDE_LIST)
SET(ECLIPSE_INCLUDE_LIST "${PART_ECLIPSE_INCLUDE_LIST}")
GEN_ECLIPSE_INCLUDE_LIST("${LIB_ARCH_INCS}" PART_ECLIPSE_INCLUDE_LIST)
SET(ECLIPSE_INCLUDE_LIST "${PART_ECLIPSE_INCLUDE_LIST} ${ECLIPSE_INCLUDE_LIST}")

GET_PROPERTY(FLAG_LIST DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY COMPILE_DEFINITIONS)
GEN_ECLIPSE_FLAG_LIST("${FLAG_LIST}" ECLIPSE_FLAG_LIST)

CONFIGURE_FILE(${ARCH_TOOLS_DIR}/eclipse/libproject.in ${PROJECT_BINARY_DIR}/.project @ONLY)
CONFIGURE_FILE(${ARCH_TOOLS_DIR}/eclipse/libcproject.in ${PROJECT_BINARY_DIR}/.cproject @ONLY)