rosbuild_add_compile_flags(${PROJECT_NAME} "-fPIC")

file(GLOB TEST_SRCS *.cpp)
controlit_build_add_test(RAPidConstraint_Test ${TEST_SRCS})
target_link_libraries(${TEST_NAME} ${PROJECT_NAME})