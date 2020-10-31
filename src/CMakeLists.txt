# Copyright 2012-2020 Osmocom Project
#
# This file is part of rtl-sdr
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

########################################################################
# Setup shared library variant
########################################################################
add_library(rtlsdr SHARED librtlsdr.c
  tuner_e4k.c tuner_fc0012.c tuner_fc0013.c tuner_fc2580.c tuner_r82xx.c)
target_link_libraries(rtlsdr ${LIBUSB_LIBRARIES} ${THREADS_PTHREADS_LIBRARY})
target_include_directories(rtlsdr PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  # <prefix>/include
  ${LIBUSB_INCLUDE_DIRS}
  ${THREADS_PTHREADS_INCLUDE_DIR}
  )
set_target_properties(rtlsdr PROPERTIES DEFINE_SYMBOL "rtlsdr_EXPORTS")
set_target_properties(rtlsdr PROPERTIES OUTPUT_NAME rtlsdr)
set_target_properties(rtlsdr PROPERTIES SOVERSION ${MAJOR_VERSION})
set_target_properties(rtlsdr PROPERTIES VERSION ${LIBVER})
generate_export_header(rtlsdr)

########################################################################
# Setup static library variant
########################################################################
add_library(rtlsdr_static STATIC librtlsdr.c
  tuner_e4k.c tuner_fc0012.c tuner_fc0013.c tuner_fc2580.c tuner_r82xx.c)
target_link_libraries(rtlsdr ${LIBUSB_LIBRARIES} ${THREADS_PTHREADS_LIBRARY})
target_include_directories(rtlsdr_static PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  # <prefix>/include
  ${LIBUSB_INCLUDE_DIRS}
  ${THREADS_PTHREADS_INCLUDE_DIR}
  )
set_property(TARGET rtlsdr_static APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
if(NOT WIN32)
# Force same library filename for static and shared variants of the library
set_target_properties(rtlsdr_static PROPERTIES OUTPUT_NAME rtlsdr)
endif()
generate_export_header(rtlsdr_static)

########################################################################
# Set up Windows DLL resource files
########################################################################
IF(MSVC)
    include(${CMAKE_SOURCE_DIR}/cmake/Modules/Version.cmake)

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/rtlsdr.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/rtlsdr.rc
    @ONLY)
  target_sources(rtlsdr PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rtlsdr.rc)
  target_sources(rtlsdr_static PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rtlsdr.rc)
ENDIF(MSVC)

########################################################################
# Setup libraries used in executables
########################################################################
add_library(convenience_static STATIC
    convenience/convenience.c
)
target_include_directories(convenience_static
  PRIVATE ${CMAKE_SOURCE_DIR}/include)
if(WIN32)
add_library(libgetopt_static STATIC
    getopt/getopt.c
)
target_link_libraries(convenience_static
    rtlsdr
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
add_executable(rtl_biast rtl_biast.c)
set(INSTALL_TARGETS rtlsdr rtlsdr_static rtl_sdr rtl_tcp rtl_test rtl_fm rtl_eeprom rtl_adsb rtl_power rtl_biast)

target_link_libraries(rtl_sdr rtlsdr convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_tcp rtlsdr convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_test rtlsdr convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_fm rtlsdr convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_eeprom rtlsdr convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_adsb rtlsdr convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_power rtlsdr convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_biast rtlsdr convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
if(UNIX)
target_link_libraries(rtl_fm m)
target_link_libraries(rtl_adsb m)
target_link_libraries(rtl_power m)
if(APPLE OR CMAKE_SYSTEM MATCHES "OpenBSD")
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
target_link_libraries(rtl_biast libgetopt_static)
set_property(TARGET rtl_sdr APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_tcp APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_test APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_fm APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_eeprom APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_adsb APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_power APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_biast APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
endif()
########################################################################
# Install built library files & utilities
########################################################################
install(TARGETS rtlsdr EXPORT RTLSDR-export
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} # .so/.dylib file
  )
install(TARGETS rtlsdr_static EXPORT RTLSDR-export
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} # .so/.dylib file
  )
install(TARGETS rtl_sdr rtl_tcp rtl_test rtl_fm rtl_eeprom rtl_adsb rtl_power rtl_biast
  DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
