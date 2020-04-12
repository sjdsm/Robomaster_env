#!/usr/bin/env python
#coding=utf-8
import numpy as np
from enum import Enum

MAP_LENGTH = 8.1
MAP_WIDTH = 4.5
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
            if pointy > self.y[0] and pointy < self.y[1]:
                return True
        return False

    def set_type(self, type):
        self.type = type

def build_map():
    # TODO: 把障碍物加进去， 包括地图四个边缘
    obstacle_list = []
    obstacles = (   # edge
                    [0, 0, 0.01, 4.5],
                    [0, 0, 8.1, 0.01],
                    [0, 4.49, 8.1, 0.01],
                    [8.09, 0, 0.01, 4.5],
                    # obstacles
                    [1.51, 0.01, 0.2, 1.0],
                    [6.39, 3.49, 0.2, 1.0],
                    [3.55, 0.945, 1.0, 0.2],
                    [3.55, 3.555, 1.0, 0.2],
                    [0.1, 3.29, 1, 0.2],
                    [7.09, 1.01, 1, 0.2],
                    [1.51, 2.15, 0.68, 0.2],
                    [5.91, 2.15, 0.68, 0.2],
                    [3.9, 2.1, 0.3, 0.3]
                )
    for location in obstacles:
        obstacle_list.append(Rectangle(location))
    #TODO: 把加成区加进去
    f_list = []
    fs = ([0.24, 2.56, 0.54, 0.48],
        [7.32, 1.46, 0.54, 0.48],
        [1.64, 1.42, 0.54, 0.48],
        [5.92, 2.60, 0.54, 0.48],
        [3.78, 0.215, 0.54, 0.48],
        [3.78, 3.805, 0.54, 0.48]
    )
    for location in fs:
        f_list.append(Rectangle(location, type=Region.FREE))
    # all funtional areas
    boot_areas = []
    for location in ((0.01 ,0.01 ,1,1), (7.09, 0.01, 1, 1), (0.01 , 3.49 ,1,1), (7.09, 3.49, 1, 1)):
        boot_areas.append(Rectangle(location, type=Region.FREE))   # red0, red1, blue0, blue1

    return obstacle_list, f_list, boot_areas

class RM_map():
    def __init__(self):
        self.length = 8.1
        self.width = 4.5
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
