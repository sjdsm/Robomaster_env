# ICRA RoboMaster AI Challenge Simulator


> RM simulator 


## TODO
1. add relative requirements to cmakelist and package.xml (finished)
2. transfer node

## ENV
1. subscribe two topics:

     gazebo simulator: env_input.msg  

        # robots' pose & speed
        robot_input[] infos
            std_msgs/Header header
            nav_msgs/Odometry chassis_odom
            # theta, w, dw (dw is not necessary)
            geometry_msgs/Point gimbal_odom
            # depth information for shooting
            float32 laser_distance

     outside controller: vel_command_stack.msg

       # robots' speed command & shoot command
       vel_command[] vels

2. publish one topic:

     env_output.msg

        # all states of a game
        std_msgs/Header header
        robot_output[] robot_infos
            string frame_id
            nav_msgs/Odometry chassis_odom
            # theta, w, dw
            geometry_msgs/Point gimbal_odom
            bool alive
            bool movable
            bool shootable
            uint16 health
            uint16 bullets
            uint16 heat
        int8[] map


## GAZEBO

1. subscribe two topics:

     env: env_output.msg

        # usage: get robots' survival state, and movable    # or not
        data.robot_infos[i].alive
        data.robot_infos[i].movable

     outside controller: vel_command_stack.msg

        # usage: update speed
        vels[i].chassis_twist
        vels[i].gimbal_twist

2. publish one topic:

     env_input.msg

# About frame_id

    a string telling a robot's team an id:
    "BLUE 0" means the first robot of the Blue team



