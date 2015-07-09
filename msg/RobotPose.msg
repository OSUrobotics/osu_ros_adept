# This message type is for representing a robot manipulator orientation
# as well as the joint state of the robot.  Joint state can be used to 
# represent the values of the robot hand or other values which may not 
# be included in the pose information.  Pose messages can be built up
# using the PyKDL package in ROS.

# This is a combination of two basic message types, sensor_msgs.JointState
# and geometry_msgs.Pose.

# Data type, data name to be used in ROS (e.g. Pose.position, Pose.velocity)

geometry_msgs/Point position
geometry_msgs/Quaternion orientation
float64[] j_position
float64[] j_velocity

