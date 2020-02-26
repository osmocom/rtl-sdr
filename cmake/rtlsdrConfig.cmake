include(FindPkgConfig)
pkg_check_modules(LIBUSB libusb-1.0 IMPORTED_TARGET)

get_filename_component(RTLSDR_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

if(NOT TARGET rtlsdr::rtlsdr)
  include("${RTLSDR_CMAKE_DIR}/rtlsdrTargets.cmake")
endif()
