# info of a single robot from gazebo

# hearder contains robot team / id and time_stamp
# frame id: "BLUE 0" / "RED 1" refers to robot id
std_msgs/Header header
nav_msgs/Odometry chassis_odom
# theta, w, dw
geometry_msgs/Point gimbal_odom
# depth information for shooting
float32 laser_distance