import numpy as np
from enum import Enum

MAP_LENGTH = 8.1
MAP_WIDTH = 5.1
seed = 0

class REGION(Enum):
    FREE = 0
    OBSTACLE = 1
    BOOT = 2
    # functional areas:F1-F6
    REDBULLET = 3
    REDHEALTH = 4
    BLUEBULLET = 5
    BULEHEALTH = 6
    NOMOVING = 7
    NOSHOOT = 8

class Rectangle():
    def __init__(self, location, type=REGION.OBSTACLE): # location: (initx, inity, x, y)
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
    obstacle_list = []
    for location in ():
        obstacle_list.append(Rectangle(location))
    #... all the obstacles
    F_list = []
    for location in ():
        F_list.append(Rectangle(location, type=REGION.FREE)
    # all funtional areas
    boot_areas = []
    for location in ((0,0,1,1),(7.1,0,1,1),(0,4.1,1,1),(7.1,4.1,1,1)):
        boot_areas.append(Rectangle(location, type=REGION.FREE))   # red0, red1, blue0, blue1
    
    return obstacle_list, F_list, boot_areas

class RM_map():
    def __init__(self):
        self.length = 8.1
        self.width = 5.1
        self.obstacles, self.fareas, self.bootareas = build_map()

    def randomlize(self):    
        seed 1+= seed
        np.random.seed(seed)
        # random seed
        order = np.array(range(6))
        np.random.shuffle(order)
        self.fareas[order[0]].set_type(REGION.REDBULLET)
        self.fareas[order[1]].set_type(REGION.REDHEALTH)
        self.fareas[order[2]].set_type(REGION.BLUEBULLET)
        self.fareas[order[3]].set_type(REGION.BULEHEALTH)
        self.fareas[order[4]].set_type(REGION.NOMOVING)
        self.fareas[order[5]].set_type(REGION.NOSHOOT)

    def reset(self):
        self.randomlize()
