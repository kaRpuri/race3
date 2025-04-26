# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_race3_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED race3_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(race3_FOUND FALSE)
  elseif(NOT race3_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(race3_FOUND FALSE)
  endif()
  return()
endif()
set(_race3_CONFIG_INCLUDED TRUE)

# output package information
if(NOT race3_FIND_QUIETLY)
  message(STATUS "Found race3: 0.0.0 (${race3_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'race3' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${race3_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(race3_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${race3_DIR}/${_extra}")
endforeach()
