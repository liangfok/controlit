# To test the TaskFactory, I actually need to create a sample plugin library.
# In this case test_task_lib is a representative task library
rosbuild_add_library(rapid_core_example_plugin rapid_core_example_plugin.cpp)
target_link_libraries(rapid_core_example_plugin ${PROJECT_NAME})

# Now for the actual factory test
controlit_build_add_test(rapid_core_factories TaskFactoryTest.cpp
                                CompoundTaskFactoryTest.cpp
                                ConstraintSetFactoryTest.cpp)
target_link_libraries(${TEST_NAME} ${PROJECT_NAME})