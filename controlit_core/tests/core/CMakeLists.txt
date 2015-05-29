# Generate .urdf files from .xacro files
controlit_build_build_urdf(testData/ testData/)

# resource_retriever is needed by the ControlModelTest
# TODO: Get rid of these three lines once we move to Catkin
# rosbuild_find_ros_package(resource_retriever)
include_directories(${resource_retriever_PACKAGE_PATH}/include)
link_directories(${resource_retriever_PACKAGE_PATH}/lib)

# Now for the actual core tests
controlit_build_add_ros_test(${PROJECT_NAME}_core
  SRCS StringUtilityTest.cpp
       SubjectTest.cpp
       ParameterTest.cpp
       ParameterReflectionTest.cpp
       ReflectionRegistryTest.cpp
       CompoundTaskTest.cpp
       RbdlExtrasTest.cpp
       SVDTest.cpp
       ControlModelTest.cpp
       ContainerUtilityTest.cpp
       TorqueControllerTest.cpp
  LAUNCH_FILE tests/core/WBCCoreTest.test
)

target_link_libraries(${TEST_NAME} ${PROJECT_NAME})

# The following test requires that parameters be loaded into the ROS parameter server.
# Thus it uses rostest
controlit_build_add_ros_test(${PROJECT_NAME}_WBCParameterTest
                      SRCS WBCParameterTest.cpp
                      LAUNCH_FILE tests/core/WBCParameterTest.test)
target_link_libraries(${TEST_NAME} ${PROJECT_NAME})