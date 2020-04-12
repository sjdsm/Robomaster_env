#!/usr/bin/env python
import numpy as np
from enum import Enum
import math
from interval import Interval
from geometry_msgs.msg import Pose, Twist, Point

ROBOT_L = 0.8
ROBOT_W = 0.6
ROBOT_H = 0.6

def distance(point_1,point_2):
    ans = math.sqrt((point_1[0] - point_2[0]) ** 2 +
                    (point_1[1] - point_2[1]) ** 2)
    return ans

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
        self.armor_dis = math.sqrt(100**2 + 300**2)
        self.armor_angle = math.atan(1/3) # 200/2/300   armor size: 200 * 200
        self.armor_set()
    def armor_set(self):
        # armor dict: Coordinates of armor borders   #  [[left_x,left_y],[right_x,right_y], normal vector angle]  normal vector angle:[0-2pi]
        self.armor={'FRONT':[[self.chassis_pose.x+self.armor_dis*math.cos(self.chassis_pose.z+self.armor_angle),self.chassis_pose.y+self.armor_dis*math.sin(self.chassis_pose.z+self.armor_angle)],
                             [self.chassis_pose.x+self.armor_dis*math.cos(self.chassis_pose.z-self.armor_angle),self.chassis_pose.y+self.armor_dis*math.sin(self.chassis_pose.z-self.armor_angle)],
                             self.chassis_pose.z],
                    'BACK':[[self.chassis_pose.x+self.armor_dis*math.cos(self.chassis_pose.z+math.pi+self.armor_angle),self.chassis_pose.y+self.armor_dis*math.sin(self.chassis_pose.z+math.pi+self.armor_angle)],
                             [self.chassis_pose.x+self.armor_dis*math.cos(self.chassis_pose.z+math.pi-self.armor_angle),self.chassis_pose.y+self.armor_dis*math.sin(self.chassis_pose.z+math.pi-self.armor_angle)],
                             self.chassis_pose.z+math.pi if self.chassis_pose.z<math.pi else self.chassis_pose.z-math.pi],
                     'LEFT':[[self.chassis_pose.x+self.armor_dis*math.cos(self.chassis_pose.z+math.pi/2+self.armor_angle),self.chassis_pose.y+self.armor_dis*math.sin(self.chassis_pose.z+math.pi/2+self.armor_angle)],
                             [self.chassis_pose.x+self.armor_dis*math.cos(self.chassis_pose.z+math.pi/2-self.armor_angle),self.chassis_pose.y+self.armor_dis*math.sin(self.chassis_pose.z+math.pi/2-self.armor_angle)], 
                             self.chassis_pose.z+math.pi/2 if self.chassis_pose.z<3/2*math.pi else self.chassis_pose.z+math.pi/2-2*math.pi],
                    'RIGHT':[[self.chassis_pose.x+self.armor_dis*math.cos(self.chassis_pose.z-math.pi/2+self.armor_angle),self.chassis_pose.y+self.armor_dis*math.sin(self.chassis_pose.z-math.pi/2+self.armor_angle)],
                             [self.chassis_pose.x+self.armor_dis*math.cos(self.chassis_pose.z-math.pi/2-self.armor_angle),self.chassis_pose.y+self.armor_dis*math.sin(self.chassis_pose.z-math.pi/2-self.armor_angle)],
                             self.chassis_pose.z-math.pi/2 if self.chassis_pose.z>1/2*math.pi else self.chassis_pose.z-math.pi/2+2*math.pi]}
class RobotState():
    def __init__(self, team=Team.BLUE, id=0, on=False, alive=False, position=[0,0,0]): # position: (x,y,theta)
        self.on = on      # do we have this robot in our simulator
        self.alive = alive      
        self.team = team
        self.id = id    # 2 robots with num 0/1
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
        self.shoot_command = False

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
        

    def shoot(self, velocity=20):
        self.shoot_command = 0
        if self.state.can_shoot:
            for target in self.enemies:
                for key, value in target.state.pose.armor.items():
                    dis_1 = distance((self.state.pose.chassis_pose.x, self.state.pose.chassis_pose.y), value[0])
                    dis_2 = distance((self.state.pose.chassis_pose.x, self.state.pose.chassis_pose.y), value[1])
                    if abs(value[2] - self.state.pose.gimbal_pose.z) > 1 / 2 * math.pi and self.state.laser_distance >= min(dis_1, dis_2):
                        difference_left_x = value[0][0] - self.state.pose.chassis_pose.x
                        difference_left_y = value[0][1] - self.state.pose.chassis_pose.y
                        difference_right_x = value[1][0] - self.state.pose.chassis_pose.x
                        difference_right_y = value[1][1] - self.state.pose.chassis_pose.y
                        if difference_left_x >= 0 and difference_left_y >= 0:
                            interval_left = math.atan(difference_left_y / (difference_left_x + 0.000001))
                        elif difference_left_x > 0 and difference_left_y < 0:
                            interval_left = math.atan(difference_left_y / (difference_left_x + 0.000001)) + 2 * math.pi
                        else:
                            interval_left = math.atan(difference_left_y / (difference_left_x + 0.000001)) + math.pi

                        if difference_right_x > 0 and difference_right_y > 0:
                            interval_right = math.atan(difference_right_y / (difference_right_x + 0.000001))
                        elif difference_right_x > 0 and difference_right_y < 0:
                            interval_right = math.atan(difference_right_y / (difference_right_x + 0.000001)) + 2 * math.pi
                        else:
                            interval_right = math.atan(difference_right_y / (difference_right_x + 0.000001)) + math.pi
                        if self.state.pose.gimbal_pose.z in Interval(interval_left, interval_right):
                            if key is 'FRONT':
                                damage = -20
                            elif key is 'BACK':
                                damage = -60
                            else:
                                damage = -40
                            target.add_health(damage)
                            break
            self.state.bullet -= 1
            self.state.heat += velocity
            
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
