cmake_minimum_required(VERSION 2.8) 
project(falkolib)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++0x)   # Enabling c++11

SET(falkolib_RUNTIME_OUTPUT_DIRECTORY ${falkolib_SOURCE_DIR}/bin CACHE PATH "Target for the binaries")
SET(falkolib_LIBRARY_OUTPUT_DIRECTORY ${falkolib_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${falkolib_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${falkolib_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${falkolib_RUNTIME_OUTPUT_DIRECTORY})
LIST(APPEND CMAKE_MODULE_PATH ${falkolib_SOURCE_DIR}/cmake_modules)

message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

find_package(Boost)  
if (${Boost_FOUND})
  message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
  message(STATUS "Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
  message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARY_DIRS}")
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
endif(${Boost_FOUND})

find_package(Eigen3 REQUIRED)
if (${EIGEN3_FOUND})
  include_directories(${EIGEN3_INCLUDE_DIR})
  message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
else(${EIGEN3_FOUND})
  message(WARNING "Cannot find Eigen3 Library")
endif(${EIGEN3_FOUND})

message(STATUS "include dir: ${falkolib_SOURCE_DIR}/include")
message(STATUS "include dir: ${falkolib_SOURCE_DIR}/EXTERNAL")
include_directories(${falkolib_SOURCE_DIR}/include ${falkolib_SOURCE_DIR}/EXTERNAL)  

add_library(falkolib 
  src/Common/HoughSpectrum.cpp
  src/Feature/FALKOExtractor.cpp
  src/Feature/OCExtractor.cpp
  src/Feature/CGH.cpp
  src/Feature/BSC.cpp
)

add_executable(testKeypointFalko test/testKeypointFalko.cpp test/testData.cpp)
target_link_libraries(testKeypointFalko falkolib ${Boost_LIBRARIES})

add_executable(testFalkoAHT test/testFalkoAHT.cpp test/testData.cpp)
target_link_libraries(testFalkoAHT falkolib ${Boost_LIBRARIES})

add_executable(testFalkoCC test/testFalkoCC.cpp test/testData.cpp)
target_link_libraries(testFalkoCC falkolib ${Boost_LIBRARIES})

add_executable(testKeypointOC test/testKeypointOC.cpp test/testData.cpp)
target_link_libraries(testKeypointOC falkolib ${Boost_LIBRARIES}) # boost_iostreams boost_system boost_filesystem


# Option "make install": copy binaries
INSTALL(TARGETS falkolib 
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

# Option "make install": copy headers
FILE(GLOB headers_Common "${CMAKE_CURRENT_SOURCE_DIR}/include/falkolib/Common/*.h")
FILE(GLOB headers_Feature "${CMAKE_CURRENT_SOURCE_DIR}/include/falkolib/Feature/*.h")
FILE(GLOB headers_Matching "${CMAKE_CURRENT_SOURCE_DIR}/include/falkolib/Matching/*.h")
FILE(GLOB headers_mcqd "${CMAKE_CURRENT_SOURCE_DIR}/EXTERNAL/mcqd/*.h")
INSTALL(FILES ${headers_Common} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/falkolib/Common)
INSTALL(FILES ${headers_Feature} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/falkolib/Feature)
INSTALL(FILES ${headers_Matching} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/falkolib/Matching)
INSTALL(FILES ${headers_mcqd} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/mcqd)

# Option "make install": copy cmake script
FILE(GLOB cmake_script "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/falkolibConfig.cmake")
message(STATUS "cmake_script " ${cmake_script})
INSTALL(FILES ${cmake_script} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/falkolib/)
