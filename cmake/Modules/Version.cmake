# Copyright 2013 OSMOCOM Project
#
# This file is part of rtl-sdr
#
# rtl-sdr is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# rtl-sdr is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with rtl-sdr; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

if(DEFINED __INCLUDED_VERSION_CMAKE)
    return()
endif()
set(__INCLUDED_VERSION_CMAKE TRUE)

# VERSION_INFO_* variables must be provided by user
set(MAJOR_VERSION ${VERSION_INFO_MAJOR_VERSION})
set(MINOR_VERSION ${VERSION_INFO_MINOR_VERSION})
set(PATCH_VERSION ${VERSION_INFO_PATCH_VERSION})

########################################################################
# Extract the version string from git describe.
########################################################################
find_package(Git QUIET)

if(GIT_FOUND)
    message(STATUS "Extracting version information from git describe...")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --always --abbrev=4 --long
        OUTPUT_VARIABLE GIT_DESCRIBE OUTPUT_STRIP_TRAILING_WHITESPACE
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
else()
    set(GIT_DESCRIBE "v${MAJOR_VERSION}.${MINOR_VERSION}.x-xxx-xunknown")
endif()

########################################################################
# Use the logic below to set the version constants
########################################################################
if("${PATCH_VERSION}" STREQUAL "git")
    # VERSION: 3.6git-xxx-gxxxxxxxx
    # LIBVER:  3.6git
    set(VERSION "${GIT_DESCRIBE}")
    set(LIBVER  "${MAJOR_VERSION}.${MINOR_VERSION}${PATCH_VERSION}")
else()
    # This is a numbered release.
    # VERSION: 3.6.1
    # LIBVER:  3.6.1
    set(VERSION "${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION}")
    set(LIBVER "${VERSION}")
endif()
