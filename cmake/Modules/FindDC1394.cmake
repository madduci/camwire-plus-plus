#############################################################################
#
# $Id: FindDC1394.cmake 2014-03-04 13:04:42Z madduci $
# 
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Try to find libDC1394 for IEEE1394 camera. First search for libdc1394-2.x
# and if not found, search for libdc1394-1.x
# Once run this will define: 
#
# DC1394_FOUND
# DC1394_1_FOUND
# DC1394_2_FOUND
# DC1394_VERSION
# DC1394_INCLUDE_DIRS
# DC1394_1_INCLUDE_DIRS
# DC1394_2_INCLUDE_DIRS
# DC1394_LIBRARIES
# DC1394_1_LIBRARY
# DC1394_2_LIBRARY
#
# The two defines below are only useful to compile with libdc1394-2.x. In
# that case DC1394_VERSION=2. 
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindDC1394.cmake: libdc1394 only available for Unix.")
  SET(DC1394_FOUND FALSE)
ELSE(NOT UNIX)
# Search for libdc1394-1.x
  
  FIND_PATH(DC1394_1_INCLUDE_DIR libdc1394/dc1394_control.h
    $ENV{DC1394_HOME}/include
    $ENV{DC1394_DIR}/include
    /usr/include )
#MESSAGE("DBG DC1394_1_INCLUDE_DIR=${DC1394_1_INCLUDE_DIR}")

  FIND_LIBRARY(DC1394_1_LIBRARY
    NAMES dc1394_control
    PATHS
    $ENV{DC1394_HOME}/lib
    $ENV{DC1394_DIR}/lib
    /usr/lib
    )
#MESSAGE("DBG DC1394_1_LIBRARY=${DC1394_LIBRARY}")

  IF(DC1394_1_LIBRARY AND DC1394_1_INCLUDE_DIR)
    SET(DC1394_FOUND TRUE)
    SET(DC1394_1_FOUND TRUE)
    SET(DC1394_VERSION 1)
    SET(DC1394_LIBRARIES ${DC1394_1_LIBRARY})
    SET(DC1394_1_INCLUDE_DIRS ${DC1394_1_INCLUDE_DIR})
    SET(DC1394_INCLUDE_DIRS ${DC1394_1_INCLUDE_DIRS})
  ELSE(DC1394_1_LIBRARY AND DC1394_1_INCLUDE_DIR)
    SET(DC1394_FOUND FALSE)
    SET(DC1394_1_FOUND FALSE)
  ENDIF(DC1394_1_LIBRARY AND DC1394_1_INCLUDE_DIR)
  
  # Search for libdc1394-2.x
  FIND_PATH(DC1394_2_INCLUDE_DIR dc1394/control.h
    $ENV{DC1394_HOME}/include
    $ENV{DC1394_DIR}/include
    /usr/include )
  #MESSAGE("DBG DC1394_2_INCLUDE_DIR=${DC1394_2_INCLUDE_DIR}")  

  FIND_LIBRARY(DC1394_2_LIBRARY
    NAMES dc1394
    PATHS 
    $ENV{DC1394_HOME}/lib
    $ENV{DC1394_DIR}/lib
    /usr/lib
    )
  #MESSAGE("DBG DC1394_2_LIBRARY=${DC1394_2_LIBRARY}")

  IF(DC1394_2_LIBRARY AND DC1394_2_INCLUDE_DIR)

    SET(CMAKE_REQUIRED_LIBRARIES ${DC1394_2_LIBRARY})
    SET(CMAKE_REQUIRED_INCLUDES ${DC1394_2_INCLUDE_DIR})
	    
    SET(DC1394_FOUND TRUE)
    SET(DC1394_2_FOUND TRUE)

    SET(DC1394_VERSION 2)
    SET(DC1394_LIBRARIES ${DC1394_2_LIBRARY})
    SET(DC1394_2_INCLUDE_DIRS ${DC1394_2_INCLUDE_DIR})
    SET(DC1394_INCLUDE_DIRS ${DC1394_2_INCLUDE_DIRS})
  ENDIF(DC1394_2_LIBRARY AND DC1394_2_INCLUDE_DIR)
  
  ## --------------------------------

  MARK_AS_ADVANCED(
    DC1394_1_LIBRARY
    DC1394_1_INCLUDE_DIR
    DC1394_2_LIBRARY
    DC1394_2_INCLUDE_DIR
    DC1394_INCLUDE_DIR
    DC1394_LIBRARIES
    DC1394_LIBRARY
    )
ENDIF(NOT UNIX)
