# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_hands_on_kitti_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED hands_on_kitti_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(hands_on_kitti_FOUND FALSE)
  elseif(NOT hands_on_kitti_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(hands_on_kitti_FOUND FALSE)
  endif()
  return()
endif()
set(_hands_on_kitti_CONFIG_INCLUDED TRUE)

# output package information
if(NOT hands_on_kitti_FIND_QUIETLY)
  message(STATUS "Found hands_on_kitti: 0.0.0 (${hands_on_kitti_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'hands_on_kitti' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${hands_on_kitti_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(hands_on_kitti_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${hands_on_kitti_DIR}/${_extra}")
endforeach()
