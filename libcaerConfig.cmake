# libcaer
# -------
#
# cmake configuration for libcaer
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# ``libcaer::caer``
#   the shared library
# ``libcaer::caerStatic``
#   static version of the library, if enabled
#
# Result VARIABLES
# ^^^^^^^^^^^^^^^^
#
# ``libcaer_FOUND``
#   ``TRUE`` if the library and all required components were found.
#   If ``FALSE`` do not use the library.
# ``libcaer_INCLUDE_DIRS``
#   path with the headers
# ``libcaer_VERSION``
#   version as "MM.mm.pp[-rev]"
# ``libcaer_VERSION_(MAJOR|MINOR|PATCH)``
#   the version parts
#


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was libcaerConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../" ABSOLUTE)

# Use original install prefix when loaded through a "/usr move"
# cross-prefix symbolic link such as /lib -> /usr/lib.
get_filename_component(_realCurr "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
get_filename_component(_realOrig "/usr/lib/x86_64-linux-gnu/cmake/libcaer" REALPATH)
if(_realCurr STREQUAL _realOrig)
  set(PACKAGE_PREFIX_DIR "/usr")
endif()
unset(_realOrig)
unset(_realCurr)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

CMAKE_POLICY(PUSH)
CMAKE_POLICY(VERSION 3.10)

INCLUDE(${PACKAGE_PREFIX_DIR}/lib/x86_64-linux-gnu/cmake/libcaer/libcaer-exports.cmake)
SET(libcaer_INCLUDE_DIRS ${PACKAGE_PREFIX_DIR}/include)

CMAKE_POLICY(POP)
