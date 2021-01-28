#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "libcaer::caer" for configuration "Release"
set_property(TARGET libcaer::caer APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(libcaer::caer PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libcaer.so.3.3.9"
  IMPORTED_SONAME_RELEASE "libcaer.so.3"
  )

list(APPEND _IMPORT_CHECK_TARGETS libcaer::caer )
list(APPEND _IMPORT_CHECK_FILES_FOR_libcaer::caer "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libcaer.so.3.3.9" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
