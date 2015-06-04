cmake_minimum_required(VERSION 2.8.3)
project(controlit_udp)

################
## ControlIt! ##
################

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    controlit_cmake
    cmake_modules
)
# message("** controlit_cmake_DIR: " ${controlit_cmake_DIR})
list(APPEND CMAKE_MODULE_PATH ${controlit_cmake_DIR}/../../../../src/controlit/controlit_cmake/cmake)
# message("** CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

include(controlitbuild)
controlit_build_init()

find_package(Eigen REQUIRED)
find_package(Rbdl REQUIRED)


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
   CATKIN_DEPENDS roscpp controlit_cmake cmake_modules
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
file(GLOB SRCS src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SRCS})

## Declare a cpp executable
# add_executable(controlit_node src/controlit_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(controlit_node controlit_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} controlit_core)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Add the ControlIt!-specific build options and macros
# rosbuild_find_ros_package(controlit_cmake)
# list(APPEND CMAKE_MODULE_PATH ${controlit_cmake_PACKAGE_PATH}/cmake)
# include(controlitbuild)
# controlit_build_init()

# find Eigen/RBDL
# controlit_build_depend_rbdl()

# file(GLOB SRCS src/*.cpp)
# rosbuild_add_library(${PROJECT_NAME} SHARED ${SRCS})
# controlit_build_link_depends(${PROJECT_NAME})

# rosbuild_genmsg()
# rosbuild_gensrv()

# add_subdirectory(test)
