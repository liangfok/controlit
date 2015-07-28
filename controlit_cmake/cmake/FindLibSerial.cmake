# Locate LibSerial
#
# This module defines
#  LIBSERIAL_FOUND, if false, do not try to link to LibSerial
#  LIBSERIAL_LIBRARY, where to find LibSerial
#  LIBSERIAL_INCLUDE_DIR, where to find SerialPort.h

# find the LibSerial include directory
find_path(LIBSERIAL_INCLUDE_DIR SerialPort.h
          PATH_SUFFIXES include
          PATHS /usr)

# find the LibSerial library
find_library(LIBSERIAL_LIBRARY
             NAMES libserial.so
             PATH_SUFFIXES lib
             PATHS /usr/lib)

# handle the QUIETLY and REQUIRED arguments and set LIBSERIAL_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBSERIAL DEFAULT_MSG LIBSERIAL_INCLUDE_DIR LIBSERIAL_LIBRARY)
mark_as_advanced(LIBSERIAL_INCLUDE_DIR LIBSERIAL_LIBRARY)
