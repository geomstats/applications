CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

PROJECT (SAI2-MODEL)
SET (PROJECT_VERSION 0.1.0) #major version is zero for now since api will certainly change in future

SET(CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include -I/usr/local/include")

# set default build to release with debug info. mimic'ed from RBDL project.
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	MESSAGE(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
	# Set the default build, and also the possible values of build type for cmake-gui
	SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
	SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# include Eigen
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
	find_package(Eigen3 QUIET)
	# TODO: Find automatic way to find for Ubuntu 14.04
	if(NOT EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
	endif()
endif()
# include_directories(${EIGEN3_INCLUDE_DIR})

# include RBDL
set(RBDL_LOCAL_DIR ${PROJECT_SOURCE_DIR}/rbdl)
set(RBDL_INCLUDE_DIR ${RBDL_LOCAL_DIR}/include ${RBDL_LOCAL_DIR}/build/include)

# include sai2-URDF
find_package(SAI2-URDF REQUIRED)

# include Parser
SET(PARSER_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/parser)
SET(PARSER_SOURCE ${PROJECT_SOURCE_DIR}/src/parser/UrdfToSai2Model.cpp)

# include Model
SET(SAI2-MODEL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)
SET(MODEL_SOURCE ${PROJECT_SOURCE_DIR}/src/Sai2Model.cpp)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${SAI2-MODEL_INCLUDE_DIRS}
	${SAI2-URDF_INCLUDE_DIRS}
	${PARSER_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
	${RBDL_INCLUDE_DIR}
	)

# Create the library
ADD_LIBRARY (sai2-model STATIC
	${MODEL_SOURCE}
	${PARSER_SOURCE}
	)

FIND_LIBRARY(RBDL_LIBRARIES rbdl PATHS ${RBDL_LOCAL_DIR}/build)
SET (SAI2-MODEL_LIBRARIES sai2-model 
	${SAI2-URDF_LIBRARIES}
	${RBDL_LIBRARIES} 
	)

# 
# export package
# 

# export package for use from the build tree
EXPORT (TARGETS sai2-model FILE ${PROJECT_BINARY_DIR}/SAI2-MODELTargets.cmake)
EXPORT (PACKAGE SAI2-MODEL)

# configure package
SET (CONF_INCLUDE_DIRS ${SAI2-MODEL_INCLUDE_DIRS} ${RBDL_INCLUDE_DIR})
SET (CONF_LIBRARIES ${SAI2-MODEL_LIBRARIES})

# create package configuration and version files
CONFIGURE_FILE(SAI2-MODELConfig.cmake.in "${PROJECT_BINARY_DIR}/SAI2-MODELConfig.cmake" @ONLY)
INCLUDE(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SAI2-MODELConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

# add examples
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/examples)
