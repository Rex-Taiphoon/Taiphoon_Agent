# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_taiphoon_agent_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED taiphoon_agent_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(taiphoon_agent_FOUND FALSE)
  elseif(NOT taiphoon_agent_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(taiphoon_agent_FOUND FALSE)
  endif()
  return()
endif()
set(_taiphoon_agent_CONFIG_INCLUDED TRUE)

# output package information
if(NOT taiphoon_agent_FIND_QUIETLY)
  message(STATUS "Found taiphoon_agent: 0.1.0 (${taiphoon_agent_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'taiphoon_agent' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${taiphoon_agent_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(taiphoon_agent_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${taiphoon_agent_DIR}/${_extra}")
endforeach()
