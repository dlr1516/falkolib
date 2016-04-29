# - Try to find Library falkolib
# Once done, this will define
#
#  falkolib_FOUND - system has falkolib module
#  falkolib_INCLUDE_DIRS - the falkolib include directories
#  falkolib_LIBRARY_DIRS - the falkolib library directories
#  falkolib_LIBRARIES - link these to use falkolib


# Uses  directory to search mrf_segmentation directory!
set(falkolib_PREFIX_DIR /usr/local)
message(STATUS "Searching falkolib in directory ${falkolib_PREFIX_DIR}." )

# Searches include directory /usr/local/include/falkolib
find_path(falkolib_INCLUDE_DIR falkolib ${falkolib_PREFIX_DIR}/include)
message(STATUS "    falkolib_INCLUDE_DIR ${falkolib_INCLUDE_DIR}." )
set(falkolib_INCLUDE_DIRS ${falkolib_INCLUDE_DIR})
  
# Searches library librimagraph.a in /usr/local/lib
find_path(falkolib_LIBRARY_DIR librimagraph.a ${falkolib_PREFIX_DIR}/lib)
message(STATUS "    falkolib_LIBRARY_DIR ${falkolib_LIBRARY_DIR}." )
set(falkolib_LIBRARY_DIRS ${falkolib_PREFIX_DIR}/lib)

# Sets the names of library components (actually A name and A component)
find_library(falkolib_LIBRARY falkolib ${falkolib_LIBRARY_DIRS})
message(STATUS "    falkolib_LIBRARY ${falkolib_LIBRARY}." )
set(falkolib_LIBRARIES ${falkolib_LIBRARY})

if(("${falkolib_INCLUDE_DIR}" STREQUAL "falkolib_INCLUDE_DIR-NOTFOUND") OR
   ("${falkolib_LIBRARY_DIRS}" STREQUAL "falkolib_LIBRARY_DIRS-NOTFOUND") OR
   ("${falkolib_LIBRARY}" STREQUAL "falkolib_LIBRARY-NOTFOUND")
  )
  message(STATUS "Library falkolib NOT found")
  unset(falkolib_FOUND)
  unset(falkolib_INCLUDE_DIR)
  unset(falkolib_LIBRARY_DIR)
  unset(falkolib_LIBRARY)
  unset(falkolib_LIBRARIES)
endif()

mark_as_advanced(falkolib_INCLUDE_DIRS falkolib_LIBRARY_DIRS falkolib_LIBRARIES)
