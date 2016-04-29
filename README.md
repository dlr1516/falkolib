
* FALKOLib - Fast Adaptive Laser Keypoint Orientation-invariant
* Copyright (C) 2016 Fabjan Kallasi and Dario Lodi Rizzini.


OVERVIEW
-------------------------------------------------

FALKOLib is a library that implements two keypoint detectors 
and two descriptors designed for 2D LIDARs.
The two detectors are:
- FALKO (Fast Adaptive Laser Keypoint Orientation-invariant), a 
  general purpose keypoint detector which gives the name to 
  the whole library;
- OC (Orthogonal Corner), a keypoint specific for enviroments 
  with straight linear walls and architectural elements 
  arranged along orthogonal directions.
The two descriptors are:
- BSC (Binary Shape Context), a binary version of standard 
  shape context descriptor;
- CGH (Cumulative Gaussian Histogram), which represents the 
  neighborhood of a point with an histogram.

Moreover, the library provides the implementation of methods 
for keypoint/feature data association. 
In particular, the following methods have been implemented:
- NN (Nearest Neighbor): each keypoint/feature of one set 
  are associated to the nearest keypoint/features of another set;
- CCDA (Combined Constraint Data Association): features are 
  associated by finding the maximal sets of corresponding
  feature pairs, which are compatible with constraints to 
  a set;
- AHT (Affine Hough Transform): the corresponding features
  are found by voting the affine/rigid transformation 
  that overlaps them according to Hough technique.

If you use this library, please cite the following paper: 

F. Kallasi, D. Lodi Rizzini, and S. Caselli. 
Fast Keypoint Features from Laser Scanner for Robot Localization and Mapping. 
IEEE Robotics and Automation Letters (RA-L), 1(1):176-183, jan 2016. 
DOI 10.1109/LRA.2016.2517210

or the most relevant publication associated by visiting: 
http://rimlab.ce.unipr.it/FALKOLib.html


DEPENDENCIES
-------------------------------------------------

The software depends on the following external libraries

Boost >= 1.36 (submodule lexical_cast)
Eigen 3.0 

The library also requires the third party library mcqd
developed by Janez Konc (see http://www.sicmm.org/konc/), 
which has been included in folder EXTERNAL.


HOW TO COMPILE
-------------------------------------------------

Let ${falkolib_ROOT} be the install directory of your local copy 
of library falkolib. 
The following standard commands are required to compile it:

  cd ${falkolib_ROOT}
  mkdir build
  cd build
  cmake ..
  make

You can also install the library into a system directory. 
To change the install directory you must set cmake environment
variable ${CMAKE_INSTALL_PREFIX} (e.g. using command "ccmake .."
before calling "cmake .."). 
Its default value on UNIX-like/Linux systems is "/usr/local".
After compiling library falkolib, run the command:

  sudo make install

The command "sudo" is required only if ${CMAKE_INSTALL_PREFIX} 
is a system diretory managed by administrator user root.
Such command copies:
- header files of ${falkolib_ROOT}/include/falkolib to
   ${CMAKE_INSTALL_PREFIX}/include/falkolib/
- library files ${falkolib_ROOT}/lib/libfalkolib.a to
   ${CMAKE_INSTALL_PREFIX}/lib/
- cmake script ${falkolib_ROOT}/cmake_modules/falkolibConfig.cmake to
   ${CMAKE_INSTALL_PREFIX}/share/falkolib/


HOW TO USE LIBRARY falkolib IN YOUR PROJECT
-------------------------------------------------

If library falkolib has been installed in system directory "/usr/local",
then it is straighforward to use it in your projects.
You needs to add the following lines to your project as in this example:


CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(foobar)

find_package(falkolib REQUIRED)  
message(STATUS "falkolib_FOUND ${falkolib_FOUND}")
message(STATUS "falkolib_INCLUDE_DIRS ${falkolib_INCLUDE_DIRS}")
message(STATUS "falkolib_LIBRARY_DIRS ${falkolib_LIBRARY_DIRS}")
message(STATUS "falkolib_LIBRARIES ${falkolib_LIBRARIES}")

if(${falkolib_FOUND}) 
  include_directories(${falkolib_INCLUDE_DIRS})
  link_directories(${falkolib_LIBRARY_DIRS})
endif()

add_executable(foobar foobar.cpp)
target_link_libraries(foobar ${falkolib_LIBRARIES})

The above example uses the variables defined in falkolibConfig.cmake:

  falkolib_FOUND - system has falkolib module
  falkolib_INCLUDE_DIRS - the falkolib include directories
  falkolib_LIBRARY_DIRS - the falkolib library directories
  falkolib_LIBRARIES - link these to use falkolib


