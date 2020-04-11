#!/usr/bin/env python
#coding=utf-8
import threading
import rospy
from enum import Enum
import map
from robot import Armor, Team, RobotState, Robot, RobotPose
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int8

import time
# from geometry_msgs.msg import Pose, Twist, Point
# from nav_msgs.msg import Odometry
from robomaster_env.msg import env_input, env_output, vel_command_stack, robot_output

import Tkinter as tk

DURATION = 180 # length of a game
EPOCH = 0.1
MAX_LASER_DISTANCE = 1000

def thread_job():
    rospy.spin()

def thread_ui(game):
    ui = UI(game)
    ui.blocked_run()
    

class RMAI_GAME():

    def __init__(self):

        global EPOCH

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
                        ("BLUE", 1): Robot(Team.BLUE, id=1)}
        
        # set initial position in boot areas

        for i, j in enumerate(self.robots):
            boot = self.map.bootareas[i]
            self.robots[j].state.pose.chassis_pose = Point((boot.x[0] + boot.x[1]) / 2, (boot.y[0] + boot.y[1]) / 2, 0)
        
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

        rospy.init_node('sim_env', anonymous=True)
        EPOCH = rospy.get_param("~epoch")
        rospy.Subscriber(rospy.get_param("~InputTopic"), env_input, self.gazebo_callback, tcp_nodelay=True)
        # command input, shooting command needed
        rospy.Subscriber(rospy.get_param("~CommandTopic"), vel_command_stack, self.vel_command_callback, tcp_nodelay=True)

        # to user
        self.info_pub = rospy.Publisher(rospy.get_param("~OutputTopic"), env_output, queue_size=10)
        # # to gazebo, robot alive or not, can move or not
        # self.condition_pub = rospy.Publisher('/Topic_param4', output_to_gazebo, queue_size=10)

        add_thread = threading.Thread(target = thread_job)
        add_thread.start()

    def step(self):
        '''
        linear_speeds: linear speed of 4 robots' chassis, default = 0
        angular_speeds: angular_speeds speed of 4 robots' chassis, default = 0
        gimbal_speeds: angular_speeds speed of 4 robots' gimbal, default = 0
        shoot_commands: [{'target':i,'velocity':num},{},{},{}]  shooting commands of 4 robots, containing shooting direction(0-360), default = -100 means no shooting
        '''
        # 1. send velocities to physical simulator

        # 2. update everything (using gazebo callback)
        self.time = time.time()

        # 3. reset map if needed
        self.passed_time = time.time()-self.start_time
        if self.passed_time // 60 == 1 or 2:
            self.map.randomlize()
        
        # 4. step robot
        self.red_alive = False
        self.blue_alive = False

        for key, robo in self.robots.items():
       
            # 4.0 survival state
            if robo.state.alive == False:
                continue
            elif key[0] == 'BLUE':
                self.blue_alive = True
            else:
                self.red_alive = True

            # # position update, done in callback
            
            # update armor
            robo.state.pose.armor_set()

            # 4.1 funcional areas
            for f in self.map.fareas:
                if f.inside(robo.state.pose.chassis_pose.x, robo.state.pose.chassis_pose.y):
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
                    elif f.type == map.Region.BULEHEALTH:
                        self.robots[('BLUE', 0)].add_health()
                        f.set_type(map.Region.FREE)
                    elif f.type == map.Region.NOMOVING:
                        robo.disable_moving(self.time)
                        f.set_type(map.Region.FREE)
                    else:
                        robo.disable_shooting(self.time)
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
            if robo.shoot_command and robo.state.laser_distance < MAX_LASER_DISTANCE:
                robo.shoot()                       

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
            new_heat = robo.state.heat - cooldown_value * EPOCH
            robo.state.heat = max(new_heat, 0)
           
        self.publish_all()

        done = self.done()

        # ignore: collision punishment

        #print("1: ", time.time() - self.time)
        return done

    def done(self):
        if self.passed_time >= DURATION:
            return True

        if (not self.red_alive) or not (self.blue_alive):
            return True       

    def reset(self):
        self.time = time.time()
        self.passed_time = 0
        self.map.reset()
                       
        self.robots = {("RED", 0): Robot(Team.RED, id=0), 
                       ("RED", 1): Robot(Team.RED, id=1), 
                       ("BLUE", 0): Robot(Team.BLUE, id=0), 
                       ("BLUE", 1): Robot(Team.BLUE, id=1)}
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

    # def publish_vel_command(self, data):
    #     self.condition_pub.publish(data)

    def publish_info(self, data):
        self.info_pub.publish(data)

    def publish_all(self):
        # vel_info = vel_command_stack()
        info = env_output()
        info.map = []
        info.robot_infos = []
        for i in self.map.fareas:
            info.map.append(int((i.type.value - 2)))

        for key, robot in self.robots.items():

            robot_key = key[0] + ' ' + str(int(key[1]))

            robot_info = robot_output()
            robot_info.frame_id = robot_key

            robot_info.alive = robot.state.alive
            robot_info.movable = robot.state.can_move
            robot_info.shootable = robot.state.can_shoot

            robot_info.health = robot.state.health
            robot_info.bullets = robot.state.bullet
            robot_info.heat = robot.state.heat

            robot_info.chassis_odom.pose.pose.position = robot.state.pose.chassis_pose
            # print(robot_info.chassis_odom.pose.pose.position)
            robot_info.chassis_odom.twist.twist = robot.state.pose.chassis_speed
            robot_info.gimbal_odom = robot.state.pose.gimbal_pose

            info.robot_infos.append(robot_info)
        
        self.publish_info(info)


class UI:

    flag_time_to_exit = False
    flag_already_exit = False

    def thread_update_UI_val(self):

        while not self.flag_time_to_exit:

            self.R0_Health_str.set( self.game.robots[('RED', 0)].state.health )
            self.R1_Health_str.set( self.game.robots[('RED', 1)].state.health )
            self.B0_Health_str.set( self.game.robots[('BLUE', 0)].state.health )
            self.B1_Health_str.set( self.game.robots[('BLUE', 1)].state.health )

            self.R0_Bullet_str.set( self.game.robots[('RED', 0)].state.bullet )
            self.R1_Bullet_str.set( self.game.robots[('RED', 1)].state.bullet )
            self.B0_Bullet_str.set( self.game.robots[('BLUE', 0)].state.bullet )
            self.B1_Bullet_str.set( self.game.robots[('BLUE', 1)].state.bullet )

            self.bt_R0_Move_str.set( self.game.robots[('RED', 0)].state.can_move )
            self.bt_R1_Move_str.set( self.game.robots[('RED', 1)].state.can_move )
            self.bt_B0_Move_str.set( self.game.robots[('BLUE', 0)].state.can_move )
            self.bt_B1_Move_str.set( self.game.robots[('BLUE', 1)].state.can_move )

            self.bt_R0_Shoot_str.set( self.game.robots[('RED', 0)].state.can_shoot )
            self.bt_R1_Shoot_str.set( self.game.robots[('RED', 1)].state.can_shoot )
            self.bt_B0_Shoot_str.set( self.game.robots[('BLUE', 0)].state.can_shoot )
            self.bt_B1_Shoot_str.set( self.game.robots[('BLUE', 1)].state.can_shoot )


            time.sleep(0.04)

        self.flag_already_exit = True


    def quit_cb(self):
        self.flag_time_to_exit = True
        while not self.flag_already_exit:
            pass
        self.window.destroy()


    def reset_cb(self):
        self.game.reset()


    def r0_health_cb(self):
        try:
            self.game.robots[('RED', 0)].set_health( int(float(self.set_R0_Health_str.get())) )
        except:
            pass


    def r1_health_cb(self):
        try:
            self.game.robots[('RED', 1)].set_health( int(float(self.set_R1_Health_str.get())) )
        except:
            pass


    def b0_health_cb(self):
        try:
            self.game.robots[('BLUE', 0)].set_health( int(float(self.set_B0_Health_str.get())) )
        except:
            pass


    def b1_health_cb(self):
        try:
            self.game.robots[('BLUE', 1)].set_health( int(float(self.set_B1_Health_str.get())) )
        except:
            pass


    def r0_bullet_cb(self):
        try:
            self.game.robots[('RED', 0)].set_bullet( int(float(self.set_R0_Bullet_str.get())) )
        except:
            pass


    def r1_bullet_cb(self):
        try:
            self.game.robots[('RED', 1)].set_bullet( int(float(self.set_R1_Bullet_str.get())) )
        except:
            pass


    def b0_bullet_cb(self):
        try:
            self.game.robots[('BLUE', 0)].set_bullet( int(float(self.set_B0_Bullet_str.get())) )
        except:
            pass


    def b1_bullet_cb(self):
        try:
            self.game.robots[('BLUE', 1)].set_bullet( int(float(self.set_B1_Bullet_str.get())) )
        except:
            pass


    def r0_move_cb(self):
        if self.bt_R0_Move_str.get() == '1':
            self.game.robots[('RED', 0)].disable_moving()
        elif self.bt_R0_Move_str.get() == '0':
            self.game.robots[('RED', 0)].disdisable_moving()


    def r1_move_cb(self):
        if self.bt_R1_Move_str.get() == '1':
            self.game.robots[('RED', 1)].disable_moving()
        elif self.bt_R1_Move_str.get() == '0':
            self.game.robots[('RED', 1)].disdisable_moving()


    def b0_move_cb(self):
        if self.bt_B0_Move_str.get() == '1':
            self.game.robots[('BLUE', 0)].disable_moving()
        elif self.bt_B0_Move_str.get() == '0':
            self.game.robots[('BLUE', 0)].disdisable_moving()


    def b1_move_cb(self):
        if self.bt_B1_Move_str.get() == '1':
            self.game.robots[('BLUE', 1)].disable_moving()
        elif self.bt_B1_Move_str.get() == '0':
            self.game.robots[('BLUE', 1)].disdisable_moving()


    def r0_shoot_cb(self):
        if self.bt_R0_Shoot_str.get() == '1':
            self.game.robots[('RED', 0)].disable_shooting()
        elif self.bt_R0_Shoot_str.get() == '0':
            self.game.robots[('RED', 0)].disdisable_shooting()


    def r1_shoot_cb(self):
        if self.bt_R1_Shoot_str.get() == '1':
            self.game.robots[('RED', 1)].disable_shooting()
        elif self.bt_R1_Shoot_str.get() == '0':
            self.game.robots[('RED', 1)].disdisable_shooting()


    def b0_shoot_cb(self):
        if self.bt_B0_Shoot_str.get() == '1':
            self.game.robots[('BLUE', 0)].disable_shooting()
        elif self.bt_B0_Shoot_str.get() == '0':
            self.game.robots[('BLUE', 0)].disdisable_shooting()


    def b1_shoot_cb(self):
        if self.bt_B1_Shoot_str.get() == '1':
            self.game.robots[('BLUE', 1)].disable_shooting()
        elif self.bt_B1_Shoot_str.get() == '0':
            self.game.robots[('BLUE', 1)].disdisable_shooting()


    def __init__(self, game):

        self.game = game

        self.window = tk.Tk()
         
        self.window.title('Robomasters AI')
         
        self.window.geometry('530x160')

        tk.Button(self.window, text='Reset', font=('Arial', 12), command=self.reset_cb).grid(row=0,column=0)

        tk.Label(self.window, text='Health', font=('Arial', 12)).grid(row=0,column=2)
        tk.Label(self.window, text='Bullet', font=('Arial', 12)).grid(row=0,column=5)
        tk.Label(self.window, text='Move', font=('Arial', 12)).grid(row=0,column=7)
        tk.Label(self.window, text='Shoot', font=('Arial', 12)).grid(row=0,column=8)

        tk.Label(self.window, text='RED 0', fg='red', font=('Arial', 12)).grid(row=2,column=0)
        tk.Label(self.window, text='RED 1', fg='red', font=('Arial', 12)).grid(row=4,column=0)
        tk.Label(self.window, text='BLUE 0', fg='blue', font=('Arial', 12)).grid(row=6,column=0)
        tk.Label(self.window, text='BLUE 1', fg='blue', font=('Arial', 12)).grid(row=8,column=0)


        self.R0_Health_str = tk.StringVar()
        self.R0_Bullet_str = tk.StringVar()
        self.R1_Health_str = tk.StringVar()
        self.R1_Bullet_str = tk.StringVar()
        self.B0_Health_str = tk.StringVar()
        self.B0_Bullet_str = tk.StringVar()
        self.B1_Health_str = tk.StringVar()
        self.B1_Bullet_str = tk.StringVar()
        self.R0_Health_str.set('0')
        self.R0_Bullet_str.set('0')
        self.R1_Health_str.set('0')
        self.R1_Bullet_str.set('0')
        self.B0_Health_str.set('0')
        self.B0_Bullet_str.set('0')
        self.B1_Health_str.set('0')
        self.B1_Bullet_str.set('0')
        tk.Label(self.window, textvariable=self.R0_Health_str, font=('Arial', 12), width=6).grid(row=2,column=1)
        tk.Label(self.window, textvariable=self.R0_Bullet_str, font=('Arial', 12), width=6).grid(row=2,column=4)
        tk.Label(self.window, textvariable=self.R1_Health_str, font=('Arial', 12), width=6).grid(row=4,column=1)
        tk.Label(self.window, textvariable=self.R1_Bullet_str, font=('Arial', 12), width=6).grid(row=4,column=4)
        tk.Label(self.window, textvariable=self.B0_Health_str, font=('Arial', 12), width=6).grid(row=6,column=1)
        tk.Label(self.window, textvariable=self.B0_Bullet_str, font=('Arial', 12), width=6).grid(row=6,column=4)
        tk.Label(self.window, textvariable=self.B1_Health_str, font=('Arial', 12), width=6).grid(row=8,column=1)
        tk.Label(self.window, textvariable=self.B1_Bullet_str, font=('Arial', 12), width=6).grid(row=8,column=4)


        self.set_R0_Health_str = tk.StringVar()
        self.set_R0_Bullet_str = tk.StringVar()
        self.set_R1_Health_str = tk.StringVar()
        self.set_R1_Bullet_str = tk.StringVar()
        self.set_B0_Health_str = tk.StringVar()
        self.set_B0_Bullet_str = tk.StringVar()
        self.set_B1_Health_str = tk.StringVar()
        self.set_B1_Bullet_str = tk.StringVar()
        tk.Entry(self.window, textvariable=self.set_R0_Health_str, width=6).grid(row=2,column=2)
        tk.Entry(self.window, textvariable=self.set_R0_Bullet_str, width=6).grid(row=2,column=5)
        tk.Entry(self.window, textvariable=self.set_R1_Health_str, width=6).grid(row=4,column=2)
        tk.Entry(self.window, textvariable=self.set_R1_Bullet_str, width=6).grid(row=4,column=5)
        tk.Entry(self.window, textvariable=self.set_B0_Health_str, width=6).grid(row=6,column=2)
        tk.Entry(self.window, textvariable=self.set_B0_Bullet_str, width=6).grid(row=6,column=5)
        tk.Entry(self.window, textvariable=self.set_B1_Health_str, width=6).grid(row=8,column=2)
        tk.Entry(self.window, textvariable=self.set_B1_Bullet_str, width=6).grid(row=8,column=5)


        tk.Button(self.window, text='Set', font=('Arial', 12), command=self.r0_health_cb).grid(row=2,column=3)
        tk.Button(self.window, text='Set', font=('Arial', 12), command=self.r1_health_cb).grid(row=4,column=3)
        tk.Button(self.window, text='Set', font=('Arial', 12), command=self.b0_health_cb).grid(row=6,column=3)
        tk.Button(self.window, text='Set', font=('Arial', 12), command=self.b1_health_cb).grid(row=8,column=3)
        tk.Button(self.window, text='Set', font=('Arial', 12), command=self.r0_bullet_cb).grid(row=2,column=6)
        tk.Button(self.window, text='Set', font=('Arial', 12), command=self.r1_bullet_cb).grid(row=4,column=6)
        tk.Button(self.window, text='Set', font=('Arial', 12), command=self.b0_bullet_cb).grid(row=6,column=6)
        tk.Button(self.window, text='Set', font=('Arial', 12), command=self.b1_bullet_cb).grid(row=8,column=6)

        self.bt_R0_Move_str = tk.StringVar()
        self.bt_R1_Move_str = tk.StringVar()
        self.bt_B0_Move_str = tk.StringVar()
        self.bt_B1_Move_str = tk.StringVar()
        self.bt_R0_Shoot_str = tk.StringVar()
        self.bt_R1_Shoot_str = tk.StringVar()
        self.bt_B0_Shoot_str = tk.StringVar()
        self.bt_B1_Shoot_str = tk.StringVar()
        tk.Button(self.window, textvariable=self.bt_R0_Move_str, font=('Arial', 12), command=self.r0_move_cb, width=3).grid(row=2,column=7)
        tk.Button(self.window, textvariable=self.bt_R1_Move_str, font=('Arial', 12), command=self.r1_move_cb, width=3).grid(row=4,column=7)
        tk.Button(self.window, textvariable=self.bt_B0_Move_str, font=('Arial', 12), command=self.b0_move_cb, width=3).grid(row=6,column=7)
        tk.Button(self.window, textvariable=self.bt_B1_Move_str, font=('Arial', 12), command=self.b1_move_cb, width=3).grid(row=8,column=7)
        tk.Button(self.window, textvariable=self.bt_R0_Shoot_str, font=('Arial', 12), command=self.r0_shoot_cb, width=3).grid(row=2,column=8)
        tk.Button(self.window, textvariable=self.bt_R1_Shoot_str, font=('Arial', 12), command=self.r1_shoot_cb, width=3).grid(row=4,column=8)
        tk.Button(self.window, textvariable=self.bt_B0_Shoot_str, font=('Arial', 12), command=self.b0_shoot_cb, width=3).grid(row=6,column=8)
        tk.Button(self.window, textvariable=self.bt_B1_Shoot_str, font=('Arial', 12), command=self.b1_shoot_cb, width=3).grid(row=8,column=8)

        self.window.protocol("WM_DELETE_WINDOW", self.quit_cb)


    def blocked_run(self):

        add_thread_UI_val = threading.Thread(target = self.thread_update_UI_val)
        add_thread_UI_val.start()

        # blocked
        self.window.mainloop()

        print("UI closed")


if __name__ == "__main__":

    print ( time.time() )

    game = RMAI_GAME()

    # UI thread
    add_ui_thread = threading.Thread(target = thread_ui, args=[game,])
    add_ui_thread.start()

    while not rospy.is_shutdown():
        game.step()
        rospy.sleep(EPOCH)                   

