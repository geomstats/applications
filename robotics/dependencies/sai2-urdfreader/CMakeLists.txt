CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

PROJECT (SAI2-URDF)
SET (PROJECT_VERSION 0.1.0)# TODO: minor version bump after examples are finished since we moved from sai2-common

SET(CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include -I/usr/local/include")

# set default build to release with debug info. mimic'ed from RBDL project.
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	MESSAGE(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
	# Set the default build, and also the possible values of build type for cmake-gui
	SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
	SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

SET (SAI2-URDF_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src
	${PROJECT_SOURCE_DIR}/src/urdf/urdfdom/urdf_parser/include
	)

# include tinyxml2
set(TINYXML_LOCAL_DIR ${PROJECT_SOURCE_DIR}/tinyxml2)
set(TINYXML_INCLUDE_DIR ${TINYXML_LOCAL_DIR})

# include urdf, compiled with application
SET(URDF_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src)
SET(URDF_INCLUDE_SUB_DIR ${URDF_INCLUDE_DIR}/urdf)
SET(URDF_SOURCE ${URDF_INCLUDE_SUB_DIR}/urdfdom/urdf_parser/src/pose.cpp
		${URDF_INCLUDE_SUB_DIR}/urdfdom/urdf_parser/src/link.cpp
		${URDF_INCLUDE_SUB_DIR}/urdfdom/urdf_parser/src/graphics.cpp
		${URDF_INCLUDE_SUB_DIR}/urdfdom/urdf_parser/src/world.cpp
		${URDF_INCLUDE_SUB_DIR}/urdfdom/urdf_parser/src/model.cpp
		${URDF_INCLUDE_SUB_DIR}/urdfdom/urdf_parser/src/joint.cpp
		${URDF_INCLUDE_SUB_DIR}/boost_replacement/printf_console.cpp
		${URDF_INCLUDE_SUB_DIR}/boost_replacement/string_split.cpp
)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${URDF_INCLUDE_DIR}
	${TINYXML_INCLUDE_DIR}
)

# Create the library
ADD_LIBRARY (sai2-urdf STATIC
	${URDF_SOURCE}
)

FIND_LIBRARY(TINYXML_LIBRARIES tinyxml2 PATHS ${TINYXML_LOCAL_DIR}/build)
FIND_LIBRARY(TINYXML_LIBRARIES tinyxml2)
SET (SAI2-URDF_LIBRARIES sai2-urdf
	${TINYXML_LIBRARIES}
)

# 
# export package
# 

# export package for use from the build tree
EXPORT (TARGETS sai2-urdf FILE ${PROJECT_BINARY_DIR}/SAI2-URDFTargets.cmake)
EXPORT (PACKAGE SAI2-URDF)

# configure package
SET (CONF_INCLUDE_DIRS ${SAI2-URDF_INCLUDE_DIRS} ${TINYXML_INCLUDE_DIR})
SET (CONF_LIBRARIES ${SAI2-URDF_LIBRARIES})

# create package configuration and version files
CONFIGURE_FILE(SAI2-URDFConfig.cmake.in "${PROJECT_BINARY_DIR}/SAI2-URDFConfig.cmake" @ONLY)
INCLUDE(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SAI2-URDFConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

# add examples
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/examples)
