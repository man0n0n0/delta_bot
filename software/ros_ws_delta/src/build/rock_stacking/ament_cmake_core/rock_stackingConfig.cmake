# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rock_stacking_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rock_stacking_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rock_stacking_FOUND FALSE)
  elseif(NOT rock_stacking_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rock_stacking_FOUND FALSE)
  endif()
  return()
endif()
set(_rock_stacking_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rock_stacking_FIND_QUIETLY)
  message(STATUS "Found rock_stacking: 0.1.0 (${rock_stacking_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rock_stacking' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT rock_stacking_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rock_stacking_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${rock_stacking_DIR}/${_extra}")
endforeach()
