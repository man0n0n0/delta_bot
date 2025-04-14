#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "remote_serial::remote_serial_native" for configuration ""
set_property(TARGET remote_serial::remote_serial_native APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(remote_serial::remote_serial_native PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libremote_serial_native.a"
  )

list(APPEND _cmake_import_check_targets remote_serial::remote_serial_native )
list(APPEND _cmake_import_check_files_for_remote_serial::remote_serial_native "${_IMPORT_PREFIX}/lib/libremote_serial_native.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
