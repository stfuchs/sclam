# This message describes an observation of a landmark
# at a certain robot position (odom).
#
#   landmark_id      : should identify each landmark
#   header.frame_id  : sensor frame_id
#   header.timestamp : synchronized timestamp of landmark and robot
#   measurment       : landmark pose in sensor frame
#   robot            : robot pose in odom frame

std_msgs/Header header
uint32 landmark_id
geometry_msgs/PoseWithCovariance measurement
geometry_msgs/PoseWithCovariance robot
