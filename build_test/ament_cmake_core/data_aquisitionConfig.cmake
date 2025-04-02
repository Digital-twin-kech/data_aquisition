# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_data_aquisition_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED data_aquisition_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(data_aquisition_FOUND FALSE)
  elseif(NOT data_aquisition_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(data_aquisition_FOUND FALSE)
  endif()
  return()
endif()
set(_data_aquisition_CONFIG_INCLUDED TRUE)

# output package information
if(NOT data_aquisition_FIND_QUIETLY)
  message(STATUS "Found data_aquisition: 0.1.0 (${data_aquisition_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'data_aquisition' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${data_aquisition_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(data_aquisition_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${data_aquisition_DIR}/${_extra}")
endforeach()
