#!/usr/bin/env python
import numpy as np
from enum import Enum
from geometry_msgs.msg import Pose, Twist, Point

ROBOT_L = 0.8
ROBOT_W = 0.6
ROBOT_H = 0.6

class Armor(Enum):
    FRONT = 0
    LEFT = 1
    BACK = 2
    RIGHT = 3

class Team(Enum):
    RED = 0
    BLUE = 1

class RobotPose():
    def __init__(self, position=[0,0,0], linear_speed=[0,0], angular_speed=[0]):
        # x,y: robot center, theta: with X-axis
        self.chassis_pose = Point()
        self.chassis_speed = Twist()
        # gimbal pose: theta, w, dw
        self.gimbal_pose = Point()

class RobotState():
    def __init__(self, team=Team.BLUE, num=0, on=False, alive=False, position=[0,0,0]): # position: (x,y,theta)
        self.on = on      # do we have this robot in our simulator
        self.alive = alive      
        self.team = team
        self.number = num    # 2 robots with num 0/1
        self.health = 2000
        self.bullet = 0
        self.heat = 0
        # self.position = position
        self.can_move = True
        self.cant_move_time = 0 # the beginning time of no moving condition
        self.can_shoot = True
        self.cant_shoot_time = 0
        self.pose = RobotPose(position=position)
        self.laser_distance = 0

        self.length = 600
        self.width = 600
        self.height = 500

class Robot():
    def __init__(self, team, num=1, on=True, alive=True, position=[0,0,0]):
        self.state = RobotState(team, num, on, alive, position)
        self.ally = None
        self.enemies = []

    def kill(self):
        self.state.alive = False
        self.state.health = 0
        self.state.can_move = False
        self.state.can_shoot = False
    
    def add_bullet(self, num=100):
        if self.state.alive:
            self.state.bullet += num
        if self.ally.state.alive == True:
            self.ally.state.bullet += num

    def add_health(self, num=200):
        if self.state.alive:
            self.state.health += num
        if self.ally.state.alive == True:
            self.ally.state.bullet += num
        

    def shoot(self, velocity):
        if not self.state.can_shoot:
            return False
        self.state.bullet -= 1
        self.state.heat += velocity
        if 25 < velocity < 30:
            self.add_health(-200)
        elif 30 <= velocity <= 35:
            self.add_health(-1000)
        elif velocity > 35:
            self.add_health(-2000)
        # TODO: success rate of shooting considering distance and velocity
        return True
        
    def disable_moving(self, time):
        self.state.can_move = False
        self.state.cant_move_time = time

    def disable_shooting(self, time):
        self.state.can_shoot = False
        self.state.cant_shoot_time = time

    def disdisable_moving(self,):
        self.state.can_move = True
        self.state.cant_move_time = 0

    def disdisable_shooting(self,):
        self.state.can_shoot = True
        self.state.cant_shoot_time = 0

    #def set_pose
