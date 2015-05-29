cmake_minimum_required(VERSION 2.8.3)
project(controlit_core)

################
## ControlIt! ##
################

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    diagnostic_msgs
    cmake_modules
    controlit_cmake
    controlit_dependency_addons
    controlit_logging
    controlit_model
    geometry_msgs
    message_generation
    roscpp
    sensor_msgs
    std_msgs
    tf
)

# message("** controlit_cmake_DIR: " ${controlit_cmake_DIR})
list(APPEND CMAKE_MODULE_PATH ${controlit_cmake_DIR}/../../../../src/controlit/controlit_cmake/cmake)
# message("** CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

include(controlitbuild)
controlit_build_init()

find_package(Boost REQUIRED COMPONENTS thread filesystem program_options)
find_package(Eigen REQUIRED)
find_package(Rbdl REQUIRED)
find_package(TinyXML REQUIRED)
find_package(YamlCpp REQUIRED)
find_package(MuParser REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  HolonomicConstraint.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  getControlItParameters.srv
  getControllerConfig.srv
  get_command.srv
  get_constraint_set_jacobians.srv
  get_inertia_matrix.srv
  get_parameters.srv
  lock.srv
  tare.srv
  unlock.srv
  updateControlItParameters.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS controlit_model controlit_dependency_addons
#  DEPENDS system_lib
)

###########
## Build ##
###########

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     message(STATUS "+++ ${_variableName}=${${_variableName}}")
# endforeach()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${EIGEN_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    # ${controlit_logging_INCLUDE_DIRS}
    # ${controlit_model_INCLUDE_DIRS}
    # ${controlit_dependency_addons_INCLUDE_DIRS}
)

## Declare a cpp library
file(GLOB SRCS src/*.cpp src/parser/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SRCS})

## Declare a cpp executable
# add_executable(controlit_node src/controlit_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} controlit_core_generate_messages_cpp)
# add_dependencies(${PROJECT_NAME} controlit_core)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${MUPARSER_LIBRARIES}
    ${RBDL_LIBRARY}
    ${PROJECT_NAME}
    ${TinyXML_LIBRARIES}
    ${YAMLCPP_LIBRARY}
)

# Add the ControlIt!-specific build options and macros
# rosbuild_find_ros_package(controlit_cmake)
# list(APPEND CMAKE_MODULE_PATH ${controlit_cmake_PACKAGE_PATH}/cmake)
# include(controlitbuild)
# controlit_build_init()

# Boost dependency: boost-signals
# rosbuild_add_boost_directories()

# find Eigen/RBDL
# controlit_build_depend_rbdl()

# find muParser
# controlit_build_depend_mu_parser()

# find Yaml-cpp
# controlit_build_depend_yaml()

# #set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# #set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
# rosbuild_genmsg()
#uncomment if you have defined services
# rosbuild_gensrv()

# # add the ControlIt! core source files
# file(GLOB SRCS src/*.cpp src/parser/*.cpp)
# rosbuild_add_library(${PROJECT_NAME} SHARED ${SRCS})
# rosbuild_add_openmp_flags(${PROJECT_NAME})
# rosbuild_link_boost(${PROJECT_NAME} signals)
# controlit_build_link_depends(${PROJECT_NAME})

# # TESTS!
# add_subdirectory(tests)
