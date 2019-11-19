from enum import Enum
from map import RM_map
from robot import Armor, Team, Robot_State, Robot, Pose

DURATION = 180 # length of a game
FREQUENCY = 50

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
    def __init__(self, initx, inity, x, y, type=REGION.OBSTACLE):
        self.x = [initx, initx + x]
        self.y = [inity, inity + y]
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
    obstacle_list.append([Rectangle(0,0,1,1)])
    #... all the obstacles
    F_list = []
    F_list.append([Rectangle(0,0,1,1, type=REGION.FREE)])
    #.. all funtional areas
    boot_areas = []
    boot_areas.append([Rectangle(0,0,1,1)])
    return obstacle_list, F_list, boot_areas

class RMAI_GAME():
    def __init__(self):
        self.duration = DURATION
        self.time = 0
        self.last_time = 0
        self.map = RM_map()
        self.robot_r0 = Robot(team=Team.RED, position=self.map.bootareas[0], num=0, on=True, alive=True)
        self.robot_r1 = Robot(team=Team.RED, position=self.map.bootareas[1], num=1, on=True, alive=True)
        self.robot_b0 = Robot(team=Team.BLUE, position=self.map.bootareas[2], num=0, on=True, alive=True)
        self.robot_b1 = Robot(team=Team.BLUE, position=self.map.bootareas[3], num=1, on=True, alive=True)
        self.robots = [self.robot_r0, self.robot_r1, self.robot_b0, self.robot_b1]
        self.robots[0].ally_state = self.robots[1].state
        self.robots[1].ally_state = self.robots[0].state
        self.robots[2].ally_state = self.robots[3].state
        self.robots[3].ally_state = self.robots[2].state

    def step(self, linear_speeds, angular_speeds, gimbal_speeds, shoot_commands):
        '''
        linear_speeds: linear speed of 4 robots' chassis, default = 0
        angular_speeds: angular_speeds speed of 4 robots' chassis, default = 0
        gimbal_speeds: angular_speeds speed of 4 robots' gimbal, default = 0
        shoot_commands: shooting commands of 4 robots, containing shooting direction(0-360), default = -100 means no shooting
        '''
        # 1. send velocities to physical simulator

        # 2. update everything (using gazebo callback)

        # 3. reset map if needed
        if int(self.time / 60) > int(self.last_time / 60):
            self.map.randomlize()
        self.last_time = self.time
        
        # 4. step robot
        for i, robo in enumerate(self.robots):
            if robo.state.alive == False:
                continue

            # 4.1 funcional areas
            for f in self.map.fareas:
                if f.inside(robo.state.pose.x, robo.state.pose.y):
                    if f.type == REGION.FREE:
                        continue
                    elif f.type == REGION.REDBULLET:
                        self.robots[0].add_bullet()
                        f.set_type(REGION.FREE)
                    elif f.type == REGION.NOMOVING:
                        robo.disable_moving(self.time)
                        f.set_type(REGION.FREE)
                    #TODO: elif......
            
            # 4.2 punish state update
            if not robo.state.can_move:
                if (self.time - robo.state.cant_move_time) > 10:
                    robo.disdisable_moving()
            if not robo.state.can_shoot:
                if (self.time - robo.state.cant_shoot_time) > 10:
                    robo.disdisable_shooting()

            # 4.3 shooting
            robo.shoot(shoot_commands[i])

            # 4.4 heating damage
            if robo.state.heat > 240:
                robo.state.health -= (robo.state.heat - 240) * 4 * 10 / FREQUENCY
            if robo.state.heat > 360:
                robo.state.health -= 2000
            
        # 5 kill dead robot
        for i, robo in enumerate(self.robots):
            if robo.state.health <= 0:
                robo.kill()

        done = self.done()
        # ignore: collision punishment
        return done

    def done(self):
        if self.time >= DURATION:
            return True
        if not self.robots[0].state.alive() and not self.robots[1].state.alive():
            return True
        if not self.robots[2].state.alive() and not self.robots[3].state.alive():
            return True       

    def reset(self):
        self.time = 0
        self.last_time = 0
        self.map.reset()
        self.robot_r0 = Robot(team=Team.RED, position=self.map.bootareas[0], num=0, on=True, alive=True)
        self.robot_r1 = Robot(team=Team.RED, position=self.map.bootareas[1], num=1, on=True, alive=True)
        self.robot_b0 = Robot(team=Team.BLUE, position=self.map.bootareas[2], num=0, on=True, alive=True)
        self.robot_b1 = Robot(team=Team.BLUE, position=self.map.bootareas[3], num=1, on=True, alive=True)
        self.robots = [self.robot_r0, self.robot_r1, self.robot_b0, self.robot_b1]
        self.robots[0].ally_state = self.robots[1].state
        self.robots[1].ally_state = self.robots[0].state
        self.robots[2].ally_state = self.robots[3].state
        self.robots[3].ally_state = self.robots[2].state