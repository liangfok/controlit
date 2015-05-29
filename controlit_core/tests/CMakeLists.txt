rosbuild_add_compile_flags(${PROJECT_NAME} "-fPIC")

add_subdirectory(core)
add_subdirectory(factories)