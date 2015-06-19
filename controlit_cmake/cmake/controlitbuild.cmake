macro(controlit_build_init)

    # Set build type
    IF(NOT CMAKE_BUILD_TYPE)
        if (DEFINED ENV{CONTROLIT_BUILD_TYPE})
            SET(CMAKE_BUILD_TYPE $ENV{CONTROLIT_BUILD_TYPE})
        else (DEFINED ENV{CONTROLIT_BUILD_TYPE})
            SET(CMAKE_BUILD_TYPE RelWithDebInfo)
        endif (DEFINED ENV{CONTROLIT_BUILD_TYPE})
    ENDIF(NOT CMAKE_BUILD_TYPE)

    # Add support for C++11 features
    if(CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
    endif(CMAKE_COMPILER_IS_GNUCXX)

    # Add more ControlIt! compile flags
    if (DEFINED ENV{CONTROLIT_COMPILE_FLAGS})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} $ENV{CONTROLIT_COMPILE_FLAGS}")
    endif (DEFINED ENV{CONTROLIT_COMPILE_FLAGS})

    # Print the compiler flags
    message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

    # Make current project's path available to C preprocessor
    add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

endmacro(controlit_build_init)
