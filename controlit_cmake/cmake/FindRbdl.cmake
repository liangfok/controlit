# Locate rbdl
#
# This module defines
#  RBDL_FOUND, if false, do not try to link to rbdl
#  RBDL_LIBRARY, where to find rbdl
#  RBDL_URDF_LIBRARY, where to find the rbdl urdf library
#  RBDL_INCLUDE_DIR, where to find yaml.h
#
# By default, the dynamic libraries of rbdl will be found. To find the static ones instead,
# you must set the RBDL_STATIC_LIBRARY variable to TRUE before calling find_package(YamlCpp ...).
#
# If rbdl is not installed in a standard path, you can use the RBDL_DIR CMake variable
# to tell CMake where rbdl is.

# attempt to find static library first if this is set
if(RBDL_STATIC_LIBRARY)
    set(RBDL_STATIC librbdl.a)
    set(RBDL_URDF_STATIC librbdl_urdfreader.a)
endif()

# find the rbdl include directory
find_path(RBDL_INCLUDE_DIR rbdl/rbdl.h
          PATH_SUFFIXES include
          PATHS
          ~/Library/Frameworks/rbdl/include/
          /Library/Frameworks/rbdl/include/
          /opt/nasa/drc/include
          /usr/local/include/
          /usr/include/
          /sw/rbdl/         # Fink
          /opt/local/rbdl/  # DarwinPorts
          /opt/csw/rbdl/    # Blastwave
          /opt/rbdl/
          ${RBDL_DIR}/include/)

# find the rbdl library
find_library(RBDL_LIBRARY
             NAMES ${RBDL_STATIC} rbdl
             PATH_SUFFIXES lib64 lib
             PATHS ~/Library/Frameworks
                    /Library/Frameworks
                    /opt/nasa/drc
                    /usr/local
                    /usr
                    /sw
                    /opt/local
                    /opt/csw
                    /opt
                    ${RBDL_DIR}/lib)

# find_library(RBDL_URDF_LIBRARY
#              NAMES ${RBDL_URDF_STATIC} rbdl_urdfreader
#              PATH_SUFFIXES lib64 lib
#              PATHS ~/Library/Frameworks
#                     /Library/Frameworks
#                     /opt/nasa/drc
#                     /usr/local
#                     /usr
#                     /sw
#                     /opt/local
#                     /opt/csw
#                     /opt
#                     ${RBDL_DIR}/lib)

# setting because we don't use this anymore because we wrote our own
set(RBDL_URDF_LIBRARY)

# handle the QUIETLY and REQUIRED arguments and set RBDL_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(RBDL DEFAULT_MSG RBDL_INCLUDE_DIR RBDL_LIBRARY)
mark_as_advanced(RBDL_INCLUDE_DIR RBDL_LIBRARY)
