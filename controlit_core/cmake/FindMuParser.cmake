# - Try to find MuParser
# Once done this will define
#
#  MUPARSER_FOUND - system has MuParser
#  MUPARSER_INCLUDE_DIRS - the MuParser include directory
#  MUPARSER_LIBRARIES - Link these to use MuParser
#  MUPARSER_DEFINITIONS - Compiler switches required for using MuParser
#
#  Copyright (c) 2010 Roman Putanowicz <putanowr@l5.pk.edu.pl>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (MUPARSER_LIBRARIES AND MUPARSER_INCLUDE_DIRS)
    # in cache already
    set(MUPARSER_FOUND TRUE)
else (MUPARSER_LIBRARIES AND MUPARSER_INCLUDE_DIRS)

    if($ENV{ROS_DISTRO} STREQUAL "hydro")
        find_path(MUPARSER_INCLUDE_DIR NAMES muParser/muParser.h
           PATHS "${BASE_DIR}/include" NO_DEFAULT_PATH
        )
        find_path(MUPARSER_INCLUDE_DIR NAMES muParser/muParser.h)
    else($ENV{ROS_DISTRO} STREQUAL "hydro")
        find_path(MUPARSER_INCLUDE_DIR NAMES muParser.h
            PATHS "${BASE_DIR}/include" NO_DEFAULT_PATH
        )
        find_path(MUPARSER_INCLUDE_DIR NAMES muParser.h)
    endif($ENV{ROS_DISTRO} STREQUAL "hydro")

    find_library(MUPARSER_LIBRARY NAMES muparser PATHS "${BASE_DIR_LIB}"
                 NO_DEFAULT_PATH)
    find_library(MUPARSER_LIBRARY NAMES muparser HINTS "/usr/lib")

    set(MUPARSER_INCLUDE_DIRS
        ${MUPARSER_INCLUDE_DIR} CACHE PATH "Path to MuParser++ headers")

    set(MUPARSER_LIBRARIES
        ${MUPARSER_LIBRARY} CACHE STRING "Directories to be linked to use MuParser++")

    include(FindPackageHandleStandardArgs)
    # handle the QUIETLY and REQUIRED arguments and set MUPARSER_FOUND to TRUE
    # if all listed variables are TRUE
    find_package_handle_standard_args(MuParser  DEFAULT_MSG
                                      MUPARSER_LIBRARY MUPARSER_INCLUDE_DIR)
    if (MUPARSER_FOUND)
        message(STATUS "MuParser header files: ${MUPARSER_INCLUDE_DIR}")
        message(STATUS "MuParser library: ${MUPARSER_LIBRARY}")
    endif (MUPARSER_FOUND)
    mark_as_advanced(MUPARSER_INCLUDE_DIRS MUPARSER_LIBRARIES)
endif (MUPARSER_LIBRARIES AND MUPARSER_INCLUDE_DIRS)
