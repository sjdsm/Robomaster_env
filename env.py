#!/usr/bin/env python
import threading
from enum import Enum
import map
from robot import Armor, Team, RobotState, Robot, RobotPose

import time
# from geometry_msgs.msg import Pose, Twist, Point
# from nav_msgs.msg import Odometry
from roborts_msgs.msg import env_input, env_output, vel_command_stack, output_to_gazebo, vel_command_stack

DURATION = 180 # length of a game

FREQUENCY = 10
MAX_LASER_DISTANCE = 1000

def thread_job():
    rospy.spin()

class RMAI_GAME():
    def __init__(self):
        self.duration = DURATION
        self.start_time = time.time()
        self.passed_time = 0
        self.map = map.RM_map()
        self.reset()
        self.red_alive = True
        self.blue_alive = True

        # I trylly prefer to use dict
        self.robots = { ("RED", 0): Robot(Team.RED, id=0), 
                        ("RED", 1): Robot(Team.RED, id=1), 
                        ("BLUE", 0): Robot(Team.BLUE, id=0), 
                        ("BLUE", 1): Robot(Team.BLUE, id=1)]
        
        # set initial position in boot areas

        for i, j in enumerate(self.robots):
            boot = self.map.bootareas[i]
            self.robots[j].state.pose = Pose(position=[(boot.x[0] + boot.x[1]) / 2, (boot.y[0] + boot.y[1]) / 2, 0])
        
        self.robots[('BLUE', 0)].ally = self.robots[('BLUE', 1)]
        self.robots[('BLUE', 1)].ally = self.robots[('BLUE', 0)]
        self.robots[('RED', 0)].ally = self.robots[('RED', 1)]
        self.robots[('RED', 1)].ally = self.robots[('RED', 0)]

        self.robots[('BLUE', 0)].enemies.append(self.robots[('RED', 0)])
        self.robots[('BLUE', 0)].enemies.append(self.robots[('RED', 1)])
        self.robots[('BLUE', 1)].enemies.append(self.robots[('RED', 0)])
        self.robots[('BLUE', 1)].enemies.append(self.robots[('RED', 1)])
        self.robots[('RED', 0)].enemies.append(self.robots[('BLUE', 0)])
        self.robots[('RED', 0)].enemies.append(self.robots[('BLUE', 1)])
        self.robots[('RED', 1)].enemies.append(self.robots[('BLUE', 0)])
        self.robots[('RED', 1)].enemies.append(self.robots[('BLUE', 1)])

        rospy.init_node('gym_env', anonymous=True)

        rospy.Subscriber("/Topic_param1", env_input, self.gazebo_callback, tcp_nodelay=True)
        # command input, shooting command needed
        rospy.Subscriber("/Topic_param2", vel_command_stack, self.vel_command_callback, tcp_nodelay=True)

        # to user
        self.info_pub = rospy.Publisher('/Topic_param3', env_output, queue_size=10)
        # to gazebo, robot alive or not, can move or not
        self.condition_pub = rospy.Publisher('/Topic_param4', output_to_gazebo, queue_size=10)

        add_thread = threading.Thread(target = thread_job)
        add_stread.start()

    def step(self):# default order: red0,red1,blue0,blue1
        '''
        linear_speeds: linear speed of 4 robots' chassis, default = 0
        angular_speeds: angular_speeds speed of 4 robots' chassis, default = 0
        gimbal_speeds: angular_speeds speed of 4 robots' gimbal, default = 0
        shoot_commands: [{'target':i,'velocity':num},{},{},{}]  shooting commands of 4 robots, containing shooting direction(0-360), default = -100 means no shooting
        '''
        # 1. send velocities to physical simulator

        # 2. update everything (using gazebo callback)


        # 3. reset map if needed
        self.passed_time = time.time()-self.start_time
        if self.passed_time // 60 == 1 or 2:
            self.map.randomlize()
        
        # 4. step robot
        self.red_alive = False
        self.blue_alive = False

        for i, idx in enumerate(self.robots):
            robo = self.robots[idx]
                       
            # 4.0 survival state
            if robo.state.alive == False:
                continue
            elif idx[0] == 'BLUE':
                self.blue_alive = True
            else:
                self.red_alive = True

            # # position update, done in callback
            
            # update armor
            robo.state.pose.armor_set()

            # 4.1 funcional areas
             for f in self.map.fareas:
                if f.inside(robo.state.pose.x, robo.state.pose.y):
                    if f.type == map.Region.FREE:
                        pass
                    elif f.type == map.Region.REDBULLET:
                        self.robots[('RED', 0)].add_bullet()
                        f.set_type(map.Region.FREE)
                    elif f.type == map.Region.BLUEBULLET:
                        self.robots[('BLUE', 0)].add_bullet()
                        f.set_type(map.Region.FREE)
                    elif f.type == map.Region.REDHEALTH:
                        self.robots[('RED', 0)].add_health()
                        f.set_type(map.Region.FREE)
                    elif f.type == map.Region.BLUEHEALTH:
                        self.robots[('BLUE', 0)].add_health()
                        f.set_type(map.Region.FREE)
                    elif f.type == map.Region.NOMOVING:
                        robo.disable_moving(time.time())
                        f.set_type(map.Region.FREE)
                    else:
                        robo.disable_shooting(time.time())
                        f.set_type(map.Region.FREE) 

                    break  
            
            # 4.2 punish state update
            if not robo.state.can_move:
                if (time.time() - robo.state.cant_move_time) > 10:
                    robo.disdisable_moving()
            if not robo.state.can_shoot:
                if (self.time - robo.state.cant_shoot_time) > 10:
                    robo.disdisable_shooting()
                       
            # 4.3 shooting
            if robo.shoot() and robo.state.laser_distance < MAX_LASER_DISTANCE:
                for target in robo.enemies:
                    if math.sqrt(pow(robo.state.pose.chassis_pose.x - target.state.pose.chassis_pose.x, 2) + 
                                 pow(robo.state.pose.chassis_pose.y - target.state.pose.chassis_pose.y, 2)) > MAX_LASER_DISTANCE + 500:
                        for key,value in target.state.pose.armor.items():
                            if abs(value[2]-robo.state.pose.gimbal_angle)>1/2*math.pi:                 
                                difference_left_x = value[0][0]-robo.state.pose.chassis_pose.x 
                                difference_left_y = value[0][1]-robo.state.pose.chassis_pose.y
                                difference_right_x = value[1][0]-robo.state.pose.chassis_pose.x  
                                difference_right_y = value[1][1]-robo.state.pose.chassis_pose.y              
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
                                if robo.state.pose.gimbal_pose.theta in Interval(interval_left, interval_right):
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
            robo.state.heat = robo.state.heat > cooldown_value / FREQUENCY and robo.state.heat - cooldown_value / FREQUENCY or 0
            
        self.publish_all()

        done = self.done()
        # ignore: collision punishment
        return done

    def done(self):
        if self.passed_time >= DURATION:
            return True

        if (not self.red_alive) or not (self.blue_alive):
            return True       

    def reset(self):
        self.time = 0
        self.passed_time = 0
        self.map.reset()
                       
        self.robots = {("RED", 0): Robot(Team.RED, num=0), 
                       ("RED", 1): Robot(Team.RED, num=1), 
                       ("BLUE", 0): Robot(Team.BLUE, num=0), 
                       ("BLUE", 1): Robot(Team.BLUE, num=1)]     
#         for i, j in enumerate(self.robots):
#             boot = self.map.bootareas[i]
#             self.robots[j].state.pose = RobotPose(position=[(boot.x[0] + boot.x[1]) / 2, (boot.y[0] + boot.y[1]) / 2, 0])
        self.robots[('BLUE', 0)].ally = self.robots[('BLUE', 1)]
        self.robots[('BLUE', 1)].ally = self.robots[('BLUE', 0)]
        self.robots[('RED', 0)].ally = self.robots[('RED', 1)]
        self.robots[('RED', 1)].ally = self.robots[('RED', 0)] 
                       
        self.robots[('BLUE', 0)].enemies.append(self.robots[('RED', 0)])
        self.robots[('BLUE', 0)].enemies.append(self.robots[('RED', 1)])
        self.robots[('BLUE', 1)].enemies.append(self.robots[('RED', 0)])
        self.robots[('BLUE', 1)].enemies.append(self.robots[('RED', 1)])
        self.robots[('RED', 0)].enemies.append(self.robots[('BLUE', 0)])
        self.robots[('RED', 0)].enemies.append(self.robots[('BLUE', 1)])
        self.robots[('RED', 1)].enemies.append(self.robots[('BLUE', 0)])
        self.robots[('RED', 1)].enemies.append(self.robots[('BLUE', 1)])   
                       
    def gazebo_callback(self, data):
        '''
        update: chassis pose, twist
                gimabal pose, twist
                laser distance
        '''

        for i, msg in enumerate(data):
            header = msg.header.frame_id.split()
            robot_key = (header[0], int(header[1]))
            
            robot = self.robots.get(robot_key, None)
            if robot:
                robot.state.pose.chassis_pose = msg.chassis_odom.pose.pose.position
                robot.state.pose.chassis_speed = msg.chassis_odom.twist.twist
                robot.state.pose.gimbal_pose = msg.gimbal_odom
                robot.state.laser_distance = msg.laser_distance

    def vel_command_callback(self, data):
        '''
        update: shoot command
        '''

        for i, msg in enumerate(data):
            header = msg.header.frame_id.split()
            robot_key = (header[0], int(header[1]))       
            robot = self.robots.get(robot_key, None)

            if robot:
                robot.shoot_command = msg.shoot

    def publish_vel_command(self, data):
        self.condition_pub.publish(data)

    def publish_info(self, data):
        self.info_pub.publish(data)

    def publish_all(self):
        # vel_info = vel_command_stack()
        info = env_output()
        info.map = []
        info.robot_infos = []
        for i in self.map.fareas:
            info.map.append(Int8((i.type.value - 2)))

        for idx in self.robots:
            robot = self.robots[idx]
            robot_key = idx[0] + ' ' + str(int(idx[1]))

            robot_info = robot_output()
            robot_info.frame_id = robot_key

            robot_info.alive = robot.state.alive
            robot_info.movable = robot.state.can_move
            robot_info.shootable = robot.state.can_shoot

            robot_info.health = robot.state.health
            robot_info.bullets = robot.state.bullet
            robot_info.heat = robot.state.heat

            robot_info.chassis_odom.pose.pose.position = robot.state.pose.chassis_pose
            robot_info.chassis_odom.twist.twist = robot.state.pose.chassis_speed
            robot_info.gimbal_odom = robot.state.pose.gimbal_pose

            info.robot_infos.append(robot_info)
        
        self.publish_info(info)


if __name__ == "__main__":
    game = RMAI_GAME()
    epoch = 1 / FREQUENCY
    while not rospy.is_shutdown():
        game.step()
        rospy.sleep(epoch)                   

