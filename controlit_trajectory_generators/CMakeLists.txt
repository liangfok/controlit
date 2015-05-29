cmake_minimum_required(VERSION 2.8.3)
project(controlit_trajectory_generators)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

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
#  INCLUDE_DIRS include
#  LIBRARIES controlit
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# # Add the ControlIt!-specific build options and macros
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