# hearder contains robot id and time_stamp
std_msgs/Header header
# chassis_twist has x linear speed:x y and angular speed: z 
geometry_msgs/Twist chassis_twist
# gimbal_twist has angular speed: z
geometry_msgs/Twist gimbal_twist
# shoot or not
bool shoot