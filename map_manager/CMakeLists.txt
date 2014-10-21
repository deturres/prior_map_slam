CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(map_manager)

# The library prefix
SET(LIB_PREFIX map_manager)

SET(map_manager_C_FLAGS)
SET(map_manager_CXX_FLAGS)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# postfix, based on type
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE STRING "postfix applied to release-with-debug-information libraries")
SET(CMAKE_MINSIZEREL_POSTFIX "_s" CACHE STRING "postfix applied to minimium-size-build libraries")

# work out the postfix; required where we use OUTPUT_NAME
IF(CMAKE_BUILD_TYPE MATCHES Release)
  SET(EXE_POSTFIX)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Debug)
  SET(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  SET(EXE_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
  SET(EXE_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

# # Allow the developer to select if Dynamic or Static libraries are built
IF(MSVC) # MSVC only works if we link statically
  OPTION(BUILD_SHARED_LIBS "Build Shared Libraries" OFF)
ELSE(MSVC)
  OPTION(BUILD_SHARED_LIBS "Build Shared Libraries" ON)
ENDIF()
SET(MAP_MANAGER_LIB_TYPE STATIC)
IF(BUILD_SHARED_LIBS)
  SET(MAP_MANAGER_LIB_TYPE SHARED)
ENDIF()

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# On the Mac platform, configure the RPATH as per the install, to
# avoid the problem of loading both the built and installed versions
# of the shared targets
IF(APPLE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
  SET(CMAKE_INSTALL_RPATH "")
ENDIF(APPLE)

# Set the output directory for the build executables and libraries
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${map_manager_SOURCE_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${map_manager_SOURCE_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${map_manager_SOURCE_DIR}/bin)
IF(WIN32)
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${map_manager_SOURCE_DIR}/bin)
ELSE(WIN32)
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${map_manager_SOURCE_DIR}/lib)
ENDIF(WIN32)

# Set search directory for looking for our custom CMake scripts to
# look for Eigen3
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${map_manager_SOURCE_DIR}/../cmake_modules)

# Detect OS and define macros appropriately
IF(WIN32)
  IF(CYGWIN)
    MESSAGE(STATUS "Compiling on Cygwin")
    ADD_DEFINITIONS(-DCYGWIN)
  ELSE(CYGWIN)
    ADD_DEFINITIONS(-DWINDOWS)
    MESSAGE(STATUS "Compiling on Windows")
  ENDIF(CYGWIN)
  IF(MSVC)
    MESSAGE(STATUS "Compiling on Windows (MSVC)")
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
  ENDIF(MSVC)
ELSEIF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ELSEIF(APPLE)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on OSX")
ENDIF(WIN32)

# Compiler specific options for gcc
IF(CMAKE_COMPILER_IS_GNUCXX)
  # Generic settings for optimisation
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse4") 
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse4") 

  # OS X
  IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}") 
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}") 
  ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # Linux
  IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4.2") 
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -msse4.2") 
  ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  
  # Activate warnings
  SET(map_manager_C_FLAGS "${boss_C_FLAGS} -Wall -W")
  SET(map_manager_CXX_FLAGS "${boss_CXX_FLAGS} -Wall -W")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# Specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${map_manager_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${map_manager_C_FLAGS}")

# Find OpenMP
FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_MAP_MANAGER_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

# Set up the top-level include directories
INCLUDE_DIRECTORIES(${map_manager_SOURCE_DIR})

# For building the CHOLMOD / CSPARSE solvers
FIND_PACKAGE(SuiteSparse)
IF(CSPARSE_FOUND)
  SET(BUILD_CSPARSE OFF CACHE BOOL "Build local CSparse library")
ELSE(CSPARSE_FOUND)
  SET(BUILD_CSPARSE ON CACHE BOOL "Build local CSparse library")
  SET(CSPARSE_FOUND TRUE)
ENDIF(CSPARSE_FOUND)

# Find Eigen3
FIND_PACKAGE(Eigen3 3.1.2 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
  
# Find Opencv
FIND_PACKAGE(OpenCV 2.4.7 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# Find g2o
FIND_PACKAGE(G2O REQUIRED)
if (G2O_FOUND)
  include_directories(${G2O_INCLUDE_DIR})
endif(G2O_FOUND)

# Find g2o_frontend
FIND_PACKAGE(G2O_FRONTEND REQUIRED)
if (G2O_FRONTEND_FOUND)
  include_directories(${G2O_FRONTEND_INCLUDE_DIR})
endif(G2O_FRONTEND_FOUND)

# Include the subdirectories
ADD_SUBDIRECTORY(map_processing)
ADD_SUBDIRECTORY(map_matching)
ADD_SUBDIRECTORY(graph2d_processing)
