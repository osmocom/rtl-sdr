# Copyright 2012 OSMOCOM Project
#
# This file is part of rtl-sdr
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

MACRO(RTLSDR_APPEND_SRCS)
    LIST(APPEND rtlsdr_srcs ${ARGV})
ENDMACRO(RTLSDR_APPEND_SRCS)

RTLSDR_APPEND_SRCS(
    librtlsdr.c
    tuner_e4k.c
    tuner_fc0012.c
    tuner_fc0013.c
    tuner_fc2580.c
    tuner_r82xx.c
)

########################################################################
# Set up Windows DLL resource files
########################################################################
IF(MSVC)
    include(${CMAKE_SOURCE_DIR}/cmake/Modules/Version.cmake)

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/rtlsdr.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/rtlsdr.rc
    @ONLY)

    RTLSDR_APPEND_SRCS(${CMAKE_CURRENT_BINARY_DIR}/rtlsdr.rc)
ENDIF(MSVC)

########################################################################
# Setup shared library variant
########################################################################
add_library(rtlsdr_shared SHARED ${rtlsdr_srcs})
target_link_libraries(rtlsdr_shared ${LIBUSB_LIBRARIES})
set_target_properties(rtlsdr_shared PROPERTIES DEFINE_SYMBOL "rtlsdr_EXPORTS")
set_target_properties(rtlsdr_shared PROPERTIES OUTPUT_NAME rtlsdr)
set_target_properties(rtlsdr_shared PROPERTIES SOVERSION ${MAJOR_VERSION})
set_target_properties(rtlsdr_shared PROPERTIES VERSION ${LIBVER})

########################################################################
# Setup static library variant
########################################################################
add_library(rtlsdr_static STATIC ${rtlsdr_srcs})
target_link_libraries(rtlsdr_static ${LIBUSB_LIBRARIES})
set_property(TARGET rtlsdr_static APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
if(NOT WIN32)
# Force same library filename for static and shared variants of the library
set_target_properties(rtlsdr_static PROPERTIES OUTPUT_NAME rtlsdr)
endif()

########################################################################
# Setup libraries used in executables
########################################################################
add_library(convenience_static STATIC
    convenience/convenience.c
)

if(WIN32)
add_library(libgetopt_static STATIC
    getopt/getopt.c
)
target_link_libraries(convenience_static
    rtlsdr_shared
)
endif()

########################################################################
# Build utility
########################################################################
add_executable(rtl_sdr rtl_sdr.c)
add_executable(rtl_tcp rtl_tcp.c)
add_executable(rtl_test rtl_test.c)
add_executable(rtl_fm rtl_fm.c)
add_executable(rtl_eeprom rtl_eeprom.c)
add_executable(rtl_adsb rtl_adsb.c)
add_executable(rtl_power rtl_power.c)
set(INSTALL_TARGETS rtlsdr_shared rtlsdr_static rtl_sdr rtl_tcp rtl_test rtl_fm rtl_eeprom rtl_adsb rtl_power)

target_link_libraries(rtl_sdr rtlsdr_shared convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_tcp rtlsdr_shared convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_test rtlsdr_shared convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_fm rtlsdr_shared convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_eeprom rtlsdr_shared convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_adsb rtlsdr_shared convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_power rtlsdr_shared convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
if(UNIX)
target_link_libraries(rtl_fm m)
target_link_libraries(rtl_adsb m)
target_link_libraries(rtl_power m)
if(APPLE)
    target_link_libraries(rtl_test m)
else()
    target_link_libraries(rtl_test m rt)
endif()
endif()

if(WIN32)
target_link_libraries(rtl_sdr libgetopt_static)
target_link_libraries(rtl_tcp ws2_32 libgetopt_static)
target_link_libraries(rtl_test libgetopt_static)
target_link_libraries(rtl_fm libgetopt_static)
target_link_libraries(rtl_eeprom libgetopt_static)
target_link_libraries(rtl_adsb libgetopt_static)
target_link_libraries(rtl_power libgetopt_static)
set_property(TARGET rtl_sdr APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_tcp APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_test APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_fm APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_eeprom APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_adsb APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_power APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
endif()
########################################################################
# Install built library files & utilities
########################################################################
install(TARGETS ${INSTALL_TARGETS}
    LIBRARY DESTINATION ${LIB_INSTALL_DIR} # .so/.dylib file
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)
