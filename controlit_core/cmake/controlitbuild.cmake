macro(controlit_build_init)

    if(CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
    endif()

    if (DEFINED ENV{CONTROLIT_COMPILE_FLAGS})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} $ENV{CONTROLIT_COMPILE_FLAGS}")
    endif (DEFINED ENV{CONTROLIT_COMPILE_FLAGS})

    message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

    # Make current project's path available to C preprocessor
    add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")


    # get_filename_component(_controlit_cmake_dir ${CMAKE_CURRENT_LIST_FILE} PATH)

    # Set _controlit_cmake_dir to point to the path to package controlit_cmake
    # rosbuild_find_ros_package(controlit_cmake)
    # set(_controlit_cmake_dir ${controlit_cmake_PACKAGE_PATH})
endmacro(controlit_build_init)

# @brief Convience macro for defining a test and its sources.
# Also sets the test name to correspond with format expected by our build server.
# macro(controlit_build_add_test TEST_ROOT_NAME ${ARGN})
#     set(TEST_NAME ${TEST_ROOT_NAME}_Test)
#     rosbuild_add_gtest(${TEST_NAME} ${ARGN} ${_controlit_cmake_dir}/src/runTests.cpp)
#     rosbuild_add_gtest_build_flags(${TEST_NAME})
#     target_link_libraries(${TEST_NAME} pthread)
# endmacro(controlit_build_add_test)

# macro(controlit_build_link_depends target)
#     target_link_libraries(${target} ${RAPID_LIBS})
# endmacro(controlit_build_link_depends)

# @brief Convience macro for defining a test and its sources.
# Also sets the test name to correspond with format expected by our build server.
# macro(controlit_build_add_ros_test TEST_ROOT_NAME ${ARGN})
#     set(options OPTIONAL)
#     set(oneValueArgs LAUNCH_FILE)
#     set(multiValueArgs SRCS)
#     cmake_parse_arguments(controlit_build_add_ros_test "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

#     if(NOT "${controlit_build_add_ros_test_LAUNCH_FILE}" STREQUAL "")
#         set(TEST_INCLUDE "<include file=\"$(find ${PROJECT_NAME})/${controlit_build_add_ros_test_LAUNCH_FILE}\" />")
#     else(NOT "${controlit_build_add_ros_test_LAUNCH_FILE}" STREQUAL "")
#         set(TEST_INCLUDE "<!-- No user specified launch file -->")
#     endif(NOT "${controlit_build_add_ros_test_LAUNCH_FILE}" STREQUAL "")

#     set(TEST_NAME "${TEST_ROOT_NAME}_Ros_Test")
#     set(LOCAL_TEST_ROOT_NAME "${TEST_ROOT_NAME}")

#     set(ROS_TEST_TEMPLATE_FILE "${_controlit_cmake_dir}/launch/ros_test.launch.in")
#     set(ROS_TEST_TARGET_FILE "${CMAKE_SOURCE_DIR}/test_gen/launch/${TEST_ROOT_NAME}.test")
#     configure_file(${ROS_TEST_TEMPLATE_FILE} ${ROS_TEST_TARGET_FILE} @ONLY)

#     set(ROS_SRC_TEMPLATE_FILE "${_controlit_cmake_dir}/src/runRosTests.cpp.in")
#     set(ROS_SRC_TARGET_FILE "${CMAKE_SOURCE_DIR}/test_gen/src/run_${TEST_ROOT_NAME}.cpp")
#     configure_file(${ROS_SRC_TEMPLATE_FILE} ${ROS_SRC_TARGET_FILE} @ONLY)

#     rosbuild_add_executable(${TEST_NAME} ${controlit_build_add_ros_test_SRCS} ${ROS_SRC_TARGET_FILE})
#     rosbuild_add_gtest_build_flags(${TEST_NAME})
#     rosbuild_add_rostest(${ROS_TEST_TARGET_FILE})
# endmacro(controlit_build_add_ros_test)

# @brief A macro to generate URDF's from Xacro
# @param xacro_dir relative location of directory holding xacro files
# @param urdf_dir relative location of output directory to place urdf files
# macro(drcbuild_build_urdf ${ARGN})
# macro(controlit_build_build_urdf xacro_dir urdf_dir)
#     rosbuild_find_ros_package(xacro)
#     include(${xacro_PACKAGE_PATH}/cmake/xacroConfig-version.cmake)
#     find_package(xacro REQUIRED)

#     # Dependency files
#     file(GLOB model_xacro_files ${xacro_dir}/*.xacro)
#     #file(GLOB world_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/worlds/*.world.xacro)
#     foreach(it ${model_xacro_files})
#         get_filename_component(basepath ${it} PATH)
#         get_filename_component(basename ${it} NAME_WE)

#         message(" input xacro: " ${basepath} "/" ${basename} ".xacro")
#         set(expanded_file "${urdf_dir}${basename}.urdf")
#         #set(expanded_file "${basepath}/../urdf")
#         message(" output urdf: " ${expanded_file})

#         xacro_add_xacro_file(${it} ${expanded_file})

#         set(model_files ${model_files} ${expanded_file})
#     endforeach(it)

#     add_custom_target(urdf_files ALL DEPENDS ${model_xacro_files}
#                              COMMENT "Generating URDF files.")
# endmacro(controlit_build_build_urdf)

macro(controlit_build_depend_yaml)
    # find Yaml-cpp
    find_package(YamlCpp REQUIRED)
    include_directories(${YAMLCPP_INCLUDE_DIR})
    message(STATUS "Found yaml-cpp:")
    message(STATUS "  include: ${YAMLCPP_INCLUDE_DIR}")
    message(STATUS "  libs: ${YAMLCPP_LIBRARY}")
    list(APPEND RAPID_LIBS ${YAMLCPP_LIBRARY})
endmacro(controlit_build_depend_yaml)

# @note Since RBDL adds that matrix plugin to Eigen, these two are coupled
macro(controlit_build_depend_rbdl)
    # Find Eigen
    message(STATUS "ROS is $ENV{ROS_DISTRO}")
    if($ENV{ROS_DISTRO} STREQUAL "hydro")
        find_package(Eigen REQUIRED)
        message(STATUS "Found eigen3:")
        message(STATUS "  include: ${EIGEN_INCLUDE_DIRS}")
        message(STATUS "  definitions: ${EIGEN_DEFINITIONS}")
        include_directories(${EIGEN_INCLUDE_DIRS})
        add_definitions(${EIGEN_DEFINITIONS})
    else($ENV{ROS_DISTRO} STREQUAL "hydro")
        find_package(Eigen3 REQUIRED)
        message(STATUS "Found eigen3:")
        message(STATUS "  include: ${EIGEN3_INCLUDE_DIR}")
        message(STATUS "  definitions: ${EIGEN3_DEFINITIONS}")
        include_directories(${EIGEN3_INCLUDE_DIR})
        add_definitions(${EIGEN3_DEFINITIONS})
    endif($ENV{ROS_DISTRO} STREQUAL "hydro")

    # find RBDL
    find_package(Rbdl REQUIRED)
    include_directories(${RBDL_INCLUDE_DIR})
    message(STATUS "Found rbdl:")
    message(STATUS "  include: ${RBDL_INCLUDE_DIR}")
    message(STATUS "  libs: ${RBDL_LIBRARY}")
    list(APPEND RAPID_LIBS ${RBDL_LIBRARY})
endmacro(controlit_build_depend_rbdl)

# macro(controlit_build_depend_eigen3)
#     if($ENV{ROS_DISTRO} STREQUAL "hydro")
#         find_package(Eigen REQUIRED)
#         message(STATUS "Found eigen3:")
#         message(STATUS "  include: ${EIGEN_INCLUDE_DIRS}")
#         message(STATUS "  definitions: ${EIGEN_DEFINITIONS}")
#         include_directories(${EIGEN_INCLUDE_DIRS})
#         add_definitions(${EIGEN_DEFINITIONS})
#     else($ENV{ROS_DISTRO} STREQUAL "hydro")
#         find_package(Eigen3 REQUIRED)
#         message(STATUS "Found eigen3:")
#         message(STATUS "  include: ${EIGEN3_INCLUDE_DIR}")
#         message(STATUS "  definitions: ${EIGEN3_DEFINITIONS}")
#         include_directories(${EIGEN3_INCLUDE_DIR})
#         add_definitions(${EIGEN3_DEFINITIONS})
#     endif($ENV{ROS_DISTRO} STREQUAL "hydro")
# endmacro(controlit_build_depend_eigen3)

macro(controlit_build_depend_json)
    # Find json
    # TODO: Add a real find_package for jsoncpp
    list(APPEND RAPID_LIBS jsoncpp)
endmacro(controlit_build_depend_json)

macro(controlit_build_depend_mu_parser)
    find_package(MuParser REQUIRED)
    include_directories(${MUPARSER_INCLUDE_DIR})
    message(STATUS "Found MuParser:")
    message(STATUS "  include:${MUPARSER_INCLUDE_DIR}")
    message(STATUS "  libs: ${MUPARSER_LIBRARY}")
    list(APPEND RAPID_LIBS ${MUPARSER_LIBRARY})
endmacro(controlit_build_depend_mu_parser)

macro(controlit_build_depend_zmq)
    # find Yaml-cpp
    find_package(Zmq REQUIRED)
    include_directories(${ZMQ_INCLUDE_DIRS})
    message(STATUS "Found yaml-cpp:")
    message(STATUS "  include: ${ZMQ_INCLUDE_DIRS}")
    message(STATUS "  libs: ${ZMQ_LIBRARIES}")
    list(APPEND RAPID_LIBS ${ZMQ_LIBRARIES})
endmacro(controlit_build_depend_zmq)

macro(controlit_build_depend_tinyxml)
    find_package(TinyXML REQUIRED)
    include_directories(${TINYXML_INCLUDE_DIRS})
    message(STATUS "Found TinyXML:")
    message(STATUS "  include: ${TINYXML_INCLUDE_DIR}")
    message(STATUS "  libs: ${TINYXML_LIBRARIES}")
    list(APPEND RAPID_LIBS ${TINYXML_LIBRARIES})
endmacro(controlit_build_depend_tinyxml)

macro(controlit_build_depend_python27)
    # @todo use find_package!!!
    include_directories(/usr/include/python2.7)
    list(APPEND RAPID_LIBS python2.7)
endmacro(controlit_build_depend_python27)

# TODO: Remove this once controllerExec is gone
macro(controlit_build_depend_orocos)
    rosbuild_find_ros_package( rtt )
    set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )

    # Ensure that we are using Orocos's version of log4cpp
    find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
    include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
endmacro(controlit_build_depend_orocos)
