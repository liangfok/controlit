rosbuild_add_compile_flags(${PROJECT_NAME} "-fPIC")

include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(rbdl_robot_urdfreader)
