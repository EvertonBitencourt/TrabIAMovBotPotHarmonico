# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_phi_aria_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED phi_aria_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(phi_aria_FOUND FALSE)
  elseif(NOT phi_aria_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(phi_aria_FOUND FALSE)
  endif()
  return()
endif()
set(_phi_aria_CONFIG_INCLUDED TRUE)

# output package information
if(NOT phi_aria_FIND_QUIETLY)
  message(STATUS "Found phi_aria: 0.0.0 (${phi_aria_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'phi_aria' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${phi_aria_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(phi_aria_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${phi_aria_DIR}/${_extra}")
endforeach()
