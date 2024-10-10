# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_scan_to_image_localizer_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED scan_to_image_localizer_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(scan_to_image_localizer_FOUND FALSE)
  elseif(NOT scan_to_image_localizer_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(scan_to_image_localizer_FOUND FALSE)
  endif()
  return()
endif()
set(_scan_to_image_localizer_CONFIG_INCLUDED TRUE)

# output package information
if(NOT scan_to_image_localizer_FIND_QUIETLY)
  message(STATUS "Found scan_to_image_localizer: 0.0.0 (${scan_to_image_localizer_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'scan_to_image_localizer' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${scan_to_image_localizer_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(scan_to_image_localizer_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${scan_to_image_localizer_DIR}/${_extra}")
endforeach()
