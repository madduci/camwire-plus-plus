cmake_minimum_required (VERSION 2.8.7)

set (PACKAGE_NAME "camwire-plus")
set (LIBRARY_NAME "camwire++")

project (${LIBRARY_NAME} C CXX)

set (Camwire_VERSION_MAJOR  1)
set (Camwire_VERSION_MINOR  9)
set (Camwire_VERSION_PATCH  5)
set (Camwire_SOVERSION      2)

set (Camwire_VERSION_STRING
        ${Camwire_VERSION_MAJOR}.${Camwire_VERSION_MINOR}.${Camwire_VERSION_PATCH})

#Enables C++11 features
IF (CMAKE_COMPILER_IS_GNUCXX)
  SET (CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Werror  -Wno-sign-compare")
ELSEIF (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  SET (CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror  -Wno-sign-compare")
ELSEIF (MSVC)
  # On by default
ENDIF ()

include_directories(src)
include_directories(include)
include_directories(test)

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

# Properties of shared lib:
set_target_properties (${LIBRARY_NAME}  PROPERTIES
        VERSION ${Camwire_VERSION_STRING}
        SOVERSION ${Camwire_SOVERSION}
        SKIP_BUILD_RPATH ON)


# What to install where:
install (TARGETS ${LIBRARY_NAME} ${LIBRARY_NAME}_static DESTINATION lib)
install (FILES include/camwirebus.hpp include/camwire.hpp include/camwire_handle.hpp DESTINATION include/camwire)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
find_package(DC1394 REQUIRED)

IF(DC1394_2_FOUND)
message("libdc1394 2.x found")
add_executable(hello test/hello.cpp)
target_link_libraries(hello ${LIBRARY_NAME} ${DC1394_LIBRARIES})
ENDIF(DC1394_2_FOUND)

find_package(OpenCV 2.4.6 REQUIRED)
if(OpenCV_FOUND)
message("opencv found")
IF(DC1394_2_FOUND)
message("libdc1394 2.x found")
find_package(X11 REQUIRED)
add_executable(hello_cv test/hello_cv.cpp)
target_link_libraries(hello_cv ${LIBRARY_NAME} ${DC1394_LIBRARIES} ${OpenCV_LIBRARIES} ${X11_LIBRARY} pthread)
ENDIF(DC1394_2_FOUND)
endif(OpenCV_FOUND)

install (TARGETS hello hello_cv DESTINATION bin)

####################################
# build a CPack driven installer package

include (InstallRequiredSystemLibraries)

set (CPACK_PACKAGE_NAME  "Camwire++")
set (CPACK_PACKAGE_VENDOR  "Humboldt Universität zu Berlin")
set (CPACK_PACKAGE_CONTACT  "Michele Adduci")
set (CPACK_PACKAGE_DESCRIPTION_SUMMARY  "Camwire++: a generic C++ API for IEEE 1394 cameras")
set (CPACK_PACKAGE_VERSION  "${Camwire_VERSION_STRING}")
set (CPACK_PACKAGE_VERSION_MAJOR  "${Camwire_VERSION_MAJOR}")
set (CPACK_PACKAGE_VERSION_MINOR  "${Camwire_VERSION_MINOR}")
set (CPACK_PACKAGE_VERSION_PATCH  "${Camwire_VERSION_PATCH}")

set (CPACK_PACKAGE_FILE_NAME
        "${CMAKE_PROJECT_NAME}-${Camwire_VERSION_STRING}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
set (CPACK_PACKAGE_INSTALL_DIRECTORY  ${PACKAGE_NAME})
set (CPACK_RESOURCE_FILE_LICENSE  "${CMAKE_HOME_DIRECTORY}/COPYING")
set (CPACK_SOURCE_PACKAGE_FILE_NAME
        "${CMAKE_PROJECT_NAME}-${Camwire_VERSION_STRING}-Source")
set (CPACK_INSTALL_DIRECTORIES  "etc share")

set (CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "i686")
  set (CPACK_DEBIAN_PACKAGE_ARCHITECTURE  "i386")
endif ()
set (CPACK_DEBIAN_PACKAGE_DEPENDS  "libdc1394 >= 2.1.1")

set (CPACK_RPM_PACKAGE_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
set (CPACK_RPM_PACKAGE_REQUIRES "libdc1394 >= 2.1.1")
set (CPACK_RPM_PACKAGE_GROUP  "System Environment/Libraries")

set (CPACK_BINARY_DEB  "ON")
set (CPACK_BINARY_RPM  "OFF")
set (CPACK_BINARY_STGZ "OFF")
set (CPACK_BINARY_TBZ2 "OFF")
set (CPACK_BINARY_TGZ  "ON")
set (CPACK_BINARY_TZ   "OFF")
# set (CPACK_BINARY_ZIP  "ON")
set (CPACK_BINARY_ZIP  "OFF")  # CPack Error: archive_write_header: Filetype not supported
set (CPACK_SOURCE_TBZ2 "OFF")
set (CPACK_SOURCE_TGZ  "ON")
set (CPACK_SOURCE_TZ   "OFF")
set (CPACK_SOURCE_ZIP  "ON")

include (CPack)
####################################


