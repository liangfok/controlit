#include <pluginlib/class_list_macros.h>
#include "task_example_plugin.hpp"
#include "constraint_example_plugin.hpp"

// Defined in /opt/ros/groovy/include/pluginlib/class_list_macros.h:
//
// PLUGINLIB_EXPORT_CLASS(class_type, base_class_type)

PLUGINLIB_EXPORT_CLASS(controlit::example::SimpleTask,       controlit::Task);
PLUGINLIB_EXPORT_CLASS(controlit::example::SimpleConstraint, controlit::Constraint);
