#!/usr/bin/env python
# -*-coding: utf-8 -*-

"""
    follower2.py - Version 1.1 2013-12-20
    
    Follow a "person" by tracking the nearest object in x-y-z space.
    
    Relies on PCL ROS nodelets in the launch file to pre-filter the
    cloud on the x, y and z dimensions.
    
    Based on the follower application by Tony Pratkanis at:
    
    http://ros.org/wiki/turtlebot_follower
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""


import rospy
from geometry_msgs.msg import Twist
from zed_interfaces.msg import ObjectsStamped
from math import copysign


no=0 # 数量？
xd=0 # x ?
zd=0 # z ?

class Follower():
    def __init__(self):
        rospy.init_node("follower")
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # The goal distance (in meters) to keep between the robot and the person
        # 目标与机器人之间的目标距离（以米为单位）。机器人将尝试保持这个距离。
        #self.goal_z = rospy.get_param("~goal_z", 0.6)
        self.goal_z = rospy.get_param("~goal_z", 1.0)
        # How far away from the goal distance (in meters) before the robot reacts
        # 目标距离的容差（以米为单位）。当目标距离超出 goal_z 时，机器人开始采取行动。
        self.z_threshold = rospy.get_param("~z_threshold", 0.05)
        
        # How far away from being centered (x displacement) on the person before the robot reacts
        # 机器人在目标上的水平偏差的容差（以米为单位）。当机器人相对于目标的水平偏差超出 x_threshold 时，机器人开始采取行动。
        self.x_threshold = rospy.get_param("~x_threshold", 0.05)
        
        # How much do we weight the goal distance (z) when making a movement
        # 控制线性速度的比例因子。它决定了机器人如何响应目标距离的变化。
        self.z_scale = rospy.get_param("~z_scale", 1.0)

        # How much do we weight x-displacement of the person when making a movement  
        # 控制角速度的比例因子。它决定了机器人如何响应水平偏差的变化。      
        self.x_scale = rospy.get_param("~x_scale", 2.5)
        
        # The maximum rotation speed in radians per second 机器人允许的最大角速度（弧度/秒）。
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)
        
        # The minimum rotation speed in radians per second 机器人允许的最小角速度（弧度/秒）。
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.0)
        
        # The max linear speed in meters per second 机器人允许的最大线性速度（米/秒）。
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)
        
        # The minimum linear speed in meters per second 机器人允许的最小线性速度（米/秒）。
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)
        
        # Slow down factor when stopping 当机器人停止时，减速因子。
        self.slow_down_factor = rospy.get_param("~slow_down_factor", 0.8)

        #tracking object type (Person, Electronics, Fruit-Vegetable...)
        self.target_type=rospy.get_param("~target_type","person")

        # store zed2i camera detected people/object count
        self.person_count =0
        
        # Initialize the movement command
        self.move_cmd = Twist()
    
        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)

        # Subscribe to the zed2i objects/person detection topic 
        self.depth_subscriber = rospy.Subscriber('detected_objects', ObjectsStamped, self.set_cmd_vel, queue_size=1)
        
        # Wait for the pointcloud topic to become available
        rospy.wait_for_message('detected_objects', ObjectsStamped)

        rospy.loginfo("Ready to follow!")
        

        rospy.sleep(3)


    def set_cmd_vel(self, msg):
        global no,xd,zd
        #global xd
        #global zd
        # Initialize the centroid coordinates point count
        x = y = z = 0
        self.person_count=len(msg.objects)


        # 判断是否有人
        found_target=False
        if self.person_count > 0:
               
            for i in range (self.person_count):
                #self.person_list.append(msg.objects[i]) 
                #print(msg.objects[i])
                # 判断type是不是person
                if msg.objects[i].sublabel==self.target_type:
                    # 判断是否已跟随一个人
                    if no==0:
                        tracking_target=msg.objects[i]
                        xd=tracking_target.position[0] 
                        zd=tracking_target.position[2]
                        no=1
                        found_target=True
                        print("found first person")
                    # 根据x方向是否在半米之内
                    else:
                        if msg.objects[i].position[0]< 0.5 :
                            tracking_target=msg.objects[i]
                            xd=tracking_target.position[0]
                            found_target=True
                            no=1
                            print ("go on keepping")
                            break
                        # 如果x不在半米之内，并且是最后一个
                        else:
                            if i==(self.person_count-1):
                                #no=0
                                tracking_target=msg.objects[i]
                                xd=tracking_target.position[0]
                                zd=tracking_target.position[2]
                                no=1
                                found_target=True
                                print("found new person ")

            if found_target==True:
                #print("we are tracking specified obeject: %s "%object_type)
                pass
            else:
                #print("Specified object %s not found! We are stopping robot instead"%self.target_type)                
                self.move_cmd = Twist()
                self.cmd_vel_pub.publish(self.move_cmd)
                return
               
            #// x = -tracking_target.position[1]  
            #// y = tracking_target.position[2]
            #// z = tracking_target.position[0]

            x = tracking_target.position[0]  
            y = tracking_target.position[1]
            z = tracking_target.position[2]

            
            # Check our movement thresholds
            if (abs(z - self.goal_z) > self.z_threshold):
                if ((abs(z - self.goal_z) - self.z_threshold) > 0.7):
                    linear_speed = (z - self.goal_z) * self.z_scale*1.3
                elif ((abs(z - self.goal_z) - self.z_threshold) > 0.2):
                    linear_speed = (z - self.goal_z) * self.z_scale*1.0
                else:
                    linear_speed = (z - self.goal_z) * self.z_scale*0.3
                # Compute the angular component of the movement
                #linear_speed = (z - self.goal_z) * self.z_scale
                
                # Make sure we meet our min/max specifications
                self.move_cmd.linear.x = copysign(max(self.min_linear_speed, 
                                        min(self.max_linear_speed, abs(linear_speed))), linear_speed)
            else:
                if (self.move_cmd.linear.x>0.001): 
                    self.move_cmd.linear.x *= self.slow_down_factor
                else:
                    self.move_cmd.linear.x = 0

            # -x的原因是坐标轴相反
            if (abs(-x -0) > self.x_threshold):  
                if ((abs(-x) - self.x_threshold) > 0.1):
                    angular_speed = -x * self.x_scale*0.8
                else:
                    angular_speed = -x * self.x_scale*0.3
                
                # Make sure we meet our min/max specifications
                self.move_cmd.angular.z = copysign(max(self.min_angular_speed, 
                                        min(self.max_angular_speed, abs(angular_speed))), angular_speed)
            else:
                # Stop the rotation smoothly
                if (self.move_cmd.angular.z>0.001): 
                    self.move_cmd.angular.z *= self.slow_down_factor
                else:
                    self.move_cmd.angular.z = 0
        # 如果判断没有人       
        else:
            no=0;xd=0;zd=0
            print("following lost")
            # Stop the robot smoothly
            if (self.move_cmd.angular.z>0.001): 
                self.move_cmd.linear.x *= self.slow_down_factor
                self.move_cmd.angular.z *= self.slow_down_factor
            else:
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0

           
        if abs(self.move_cmd.linear.x) < 1e-4 and abs(self.move_cmd.angular.z)  < 1e-4:
            self.move_cmd = Twist()
            rospy.loginfo("forced to be 0")
            #pass

        # Publish the movement command
        self.cmd_vel_pub.publish(self.move_cmd)
        print("speed:", self.move_cmd)



        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        # Unregister the subscriber to stop cmd_vel publishing
        self.depth_subscriber.unregister()
        rospy.sleep(1)
        
        # Send an emtpy Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)      
                   
if __name__ == '__main__':
    rospy.loginfo("Ready to follow!")
    try:
        Follower()
        #Follower.talkback("detected %d object"%Follower.person_count)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")
