# output all states of a game

std_msgs/Header header

# robot info
robot_output[] robot_infos

# map info: usage of 6 special areas
# 0: used
# 1: red_bullet
# 2: red_health
# 3: blue_bullet
# 4: bule_health
# 5: no_move
# 6: no_shoot
uint8[]  map
