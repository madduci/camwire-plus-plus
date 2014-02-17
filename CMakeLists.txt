cmake_minimum_required (VERSION 2.8.7)

set (PACKAGE_NAME "camwire-plus-plus")
set (LIBRARY_NAME "camwire++")

project (${LIBRARY_NAME} C CXX)

set (${PACKAGE_NAME}_VERSION_MAJOR  0)
set (${PACKAGE_NAME}_VERSION_MINOR  1)
set (${PACKAGE_NAME}_VERSION_PATCH  0)
set (${PACKAGE_NAME}_SOVERSION      0)

#Enables C++11 features
IF (CMAKE_COMPILER_IS_GNUCXX)
  SET (CMAKE_CXX_FLAGS "-std=gnu++11")
ELSEIF (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  SET (CMAKE_CXX_FLAGS "-std=c++11")
ELSEIF (MSVC)
  # On by default
ENDIF ()

include_directories(src)
include_directories(include)

file(GLOB_RECURSE ${PACKAGE_NAME}_HEADERS include/*.hpp)
file(GLOB_RECURSE ${PACKAGE_NAME}_SRC src/*.cpp)
#set(${PACKAGE_NAME}_SRC src/CameraBus.cpp)


# Configure a header file to pass CMake Camwire_VERSION_* settings to the source code:
configure_file("${PROJECT_SOURCE_DIR}/include/camwire_config.hpp.in" "${PROJECT_BINARY_DIR}/include/camwire_config.hpp")

# The Camwire library targets:
add_library(${LIBRARY_NAME} SHARED ${${PACKAGE_NAME}_HEADERS} ${${PACKAGE_NAME}_SRC})
set_target_properties (${LIBRARY_NAME}  PROPERTIES
        OUTPUT_NAME ${LIBRARY_NAME}
        CLEAN_DIRECT_OUTPUT 1)

add_library(${LIBRARY_NAME}_static STATIC ${${PACKAGE_NAME}_HEADERS} ${${PACKAGE_NAME}_SRC})
set_target_properties (${LIBRARY_NAME}_static  PROPERTIES
        OUTPUT_NAME ${LIBRARY_NAME}
        CLEAN_DIRECT_OUTPUT 1)

# Support definition of Camwire's CAMERA_DEBUG:
string (TOUPPER "${CMAKE_BUILD_TYPE}" ${LIBRARY_NAME}_BUILD_TYPE_UPPER)
if ((${LIBRARY_NAME}_BUILD_TYPE_UPPER STREQUAL DEBUG) OR
    (${LIBRARY_NAME}_BUILD_TYPE_UPPER STREQUAL RELWITHDEBINFO))
  set_target_properties (${LIBRARY_NAME}  PROPERTIES
        COMPILE_DEFINITIONS CAMERA_DEBUG)
  set_target_properties (${LIBRARY_NAME}_static  PROPERTIES
        COMPILE_DEFINITIONS CAMERA_DEBUG)
endif ()

