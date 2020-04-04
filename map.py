#!/usr/bin/env python
import numpy as np
from enum import Enum

MAP_LENGTH = 8.1
MAP_WIDTH = 5.1
RANDOMSEED = 0

class Region(Enum):

    OBSTACLE = 0
    BOOT = 1
    # functional areas:F1-F6
    FREE = 2
    REDBULLET = 3
    REDHEALTH = 4
    BLUEBULLET = 5
    BULEHEALTH = 6
    NOMOVING = 7
    NOSHOOT = 8

class Rectangle():
    def __init__(self, location, type=Region.OBSTACLE): # location: (initx, inity, x, y)
        self.x = [location[0], location[0] + location[2]]
        self.y = [location[1], location[1] + location[3]]
        self.type = type

    def inside(self, pointx, pointy):
        if pointx > self.x[0] and pointx < self.x[1]:
            if pointy > self.y[0] and pointx < self.y[1]:
                return True
        return False

    def set_type(self, type):
        self.type = type

def build_map():
    # TODO: 把障碍物加进去， 包括地图四个边缘
    obstacle_list = []
    for location in ():
        obstacle_list.append(Rectangle(location))
    #TODO: 把加成区加进去
    F_list = []
    for location in ():
        F_list.append(Rectangle(location, type=Region.FREE))
    # all funtional areas
    boot_areas = []
    for location in ((0,0,1,1),(7.1,0,1,1),(0,4.1,1,1),(7.1,4.1,1,1)):
        boot_areas.append(Rectangle(location, type=Region.FREE))   # red0, red1, blue0, blue1
    
    return obstacle_list, F_list, boot_areas

class RM_map():
    def __init__(self):
        self.length = 8.1
        self.width = 5.1
        self.obstacles, self.fareas, self.bootareas = build_map()

    def randomlize(self):    
        global RANDOMSEED
        RANDOMSEED += 1

        np.random.seed(RANDOMSEED)
        # random seed
        order = np.array(range(6))
        np.random.shuffle(order)
        self.fareas[order[0]].set_type(Region.REDBULLET)
        self.fareas[order[1]].set_type(Region.REDHEALTH)
        self.fareas[order[2]].set_type(Region.BLUEBULLET)
        self.fareas[order[3]].set_type(Region.BULEHEALTH)
        self.fareas[order[4]].set_type(Region.NOMOVING)
        self.fareas[order[5]].set_type(Region.NOSHOOT)

    def reset(self):
        self.randomlize()
