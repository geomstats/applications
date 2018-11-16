# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${SAI2-MODELINCLUDE_DIRS})

# create an executable
ADD_EXECUTABLE (01-create_model_from_file main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (01-create_model_from_file
	${SAI2-MODEL_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY pbot.urdf DESTINATION ${RESOURCE_DIR})
