from enum import Enum
from map import RM_map
from robot import Armor, Team, Robot_State, Robot, Pose
import time
from interval import Interval

DURATION = 180 # length of a game
FREQUENCY = 50

class Armor(Enum):
    FRONT = 1
    LEFT = 2
    RIGHT = 2
    BACK = 3
  

class RMAI_GAME():
    def __init__(self):
        self.duration = DURATION
        self.start_time = time.time()
        self.last_time = 0
        self.map = RM_map()
        self.reset()

    def step(self, pose, shoot_commands):# default order: red0,red1,blue0,blue1
        '''
        linear_speeds: linear speed of 4 robots' chassis, default = 0
        angular_speeds: angular_speeds speed of 4 robots' chassis, default = 0
        gimbal_speeds: angular_speeds speed of 4 robots' gimbal, default = 0
        shoot_commands: [{'target':i,'velocity':num},{},{},{}]  shooting commands of 4 robots, containing shooting direction(0-360), default = -100 means no shooting
        '''
        # 1. send velocities to physical simulator

        # 2. update everything (using gazebo callback)

        # 3. reset map if needed
        self.last_time = time.time()-self.start_time
        if self.last_time // 60 == 1 or 2:
            self.map.randomlize()
        
        
        # 4. step robot
        for i, robo in enumerate(self.robots):
            if robo.state.alive == False:
                continue
            # 4.0 position update
            robo.state.pose = pose[i]

            # 4.1 funcional areas
            for f in self.map.fareas:
                if f.inside(robo.state.pose.x, robo.state.pose.y):
                    if f.type == REGION.FREE:
                        continue
                    elif f.type == REGION.REDBULLET:
                        self.robots[0].add_bullet()
                        self.robots[1].add_bullet()
                        f.set_type(REGION.FREE)
                    elif f.type == REGION.BLUEBULLET:
                        self.robots[2].add_bullet()
                        self.robots[3].add_bullet()
                        f.set_type(REGION.FREE)
                    elif f.type == REGION.REDHEALTH:
                        self.robots[0].add_health()
                        self.robots[1].add_bullet()
                        f.set_type(REGION.FREE)
                    elif f.type == REGION.BLUEHEALTH:
                        self.robots[2].add_health()
                        self.robots[3].add_bullet()
                        f.set_type(REGION.FREE)
                    elif f.type == REGION.NOMOVING:
                        robo.disable_moving(time.time())
                        f.set_type(REGION.FREE)
                    else:
                        robo.disable_shooting(time.time())
                        f.set_type(REGION.FREE)    
            
            # 4.2 punish state update
            if not robo.state.can_move:
                if (time.time() - robo.state.cant_move_time) > 10:
                    robo.disdisable_moving()
            if not robo.state.can_shoot:
                if (self.time - robo.state.cant_shoot_time) > 10:
                    robo.disdisable_shooting()

            # 4.3 shooting
            if shoot_commands[i]['velocity']>0 and robo.shoot(shoot_commands[i]['velocity']) and robo.state.pose.gimbal_laser_dis < 50:
                target = self.robots[shoot_commands[i]['taget_num']                     
                for key,value in target.state.pose.armor.items():
                    if abs(value[2]-robo.state.pose.gimbal_angle)>1/2*math.pi:                 
                        difference_left_x = value[0][0]-robo.state.pose.chassis_position[0]  
                        difference_left_y = value[0][1]-robo.state.pose.chassis_position[1]
                        difference_right_x = value[1][0]-robo.state.pose.chassis_position[0]  
                        difference_right_y = value[1][1]-robo.state.pose.chassis_position[1]                 
                        if difference_left_x>0 and difference_left_y>0:
                            interval_left= math.atan(difference_left_y/difference_left_x)
                        elif difference_left_x>0 and difference_left_y<0:
                            interval_left= math.atan(difference_left_y/difference_left_x) + 2*math.pi
                        else:
                            interval_left= math.atan(difference_left_y/difference_left_x) + math.pi

                        if difference_right_x>0 and difference_right_y>0:
                            interval_right= math.atan(difference_right_y/difference_right_x)
                        elif difference_right_x>0 and difference_right_y<0:
                            interval_right= math.atan(difference_right_y/difference_right_x) + 2*math.pi
                        else:
                            interval_right= math.atan(difference_right_y/difference_right_x) + math.pi                 
                        if robo.state.pose.gimbal_angle in Interval(interval_left, interval_right):
                            if key is 'FRONT':
                                damage = 20
                            elif key is 'BACK':
                                damage = 40     
                            else: 
                                damage = 60  
                            target.add_health(damage)                         
            
            # 4.4 health update
              # 4.4.1 heating damage
            if robo.state.heat > 240:
                robo.add_health(-(robo.state.heat - 240) * 4 ) 
            elif robo.state.heat > 360:
                robo.add_health(-(robo.state.heat - 360) * 40) 
                robo.state.heat = 360
            
            # 4.5 kill dead robot
            if robo.state.health <= 0:
                robo.kill()
                
            # 4.6 heat cooldown
            cooldown_value = 240 if robo.state.health < 400 else 120
            robo.state.heat = robo.state.heat > cooldown_valeu / FREQUENCY and robo.state.heat - cooldown_valeu / FREQUENCY or 0
            
        done = self.done()
        # ignore: collision punishment
        return done

    def done(self):
        if self.last_time >= DURATION:
            return True
        if not self.robots[0].state.alive() and not self.robots[1].state.alive():
            return True
        if not self.robots[2].state.alive() and not self.robots[3].state.alive():
            return True       

    def reset(self):
        self.time = 0
        self.last_time = 0
        self.map.reset()
        self.robot_r0 = Robot(team=Team.RED, position=self.map.bootareas[0], num=0, on=True, alive=True) # position: rectangle
        self.robot_r1 = Robot(team=Team.RED, position=self.map.bootareas[1], num=1, on=True, alive=True)
        self.robot_b0 = Robot(team=Team.BLUE, position=self.map.bootareas[2], num=0, on=True, alive=True)
        self.robot_b1 = Robot(team=Team.BLUE, position=self.map.bootareas[3], num=1, on=True, alive=True)
        self.robots = [self.robot_r0, self.robot_r1, self.robot_b0, self.robot_b1]
        self.robots[0].ally_state = self.robots[1].state
        self.robots[1].ally_state = self.robots[0].state
        self.robots[2].ally_state = self.robots[3].state
        self.robots[3].ally_state = self.robots[2].state
