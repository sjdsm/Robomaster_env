#!/usr/bin/env python
import rospy
from roborts_msg.msg import multi_output

class Blackboard():


    def __init__(self):
        rospy.Subscriber("/Topic_param1", multi_output, self.gazebo_callback, tcp_nodelay=True)

    def gazebo_callback(self, data):
        for i, msg in enumerate(data):
            