# This message represents a linear holonomic constraint, which is a constraint
# that can be expressed in the form:
#
#   jacobian * q = constant

Header header
geometry_msgs/Point point
std_msgs/Float64MultiArray jacobian
