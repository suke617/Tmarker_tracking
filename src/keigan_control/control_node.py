#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import sqrt, cos, sin
import rospy
import numpy as np
import time 
from geometry_msgs.msg import Twist
import rosparam
from std_msgs.msg import Int64MultiArray, Int64, String

limit=0.001

"""
@author 吉田圭佑
color_tracking_nodeのpubを受け取りモータの移動量を決定するプログラム

"""

class Motor_Run():  
    def __init__(self):
        self.twist=Twist()
        self.robot_data=0
        self.yaw_data=0
        self.depth=0
        self.last_depth=0
        self.last_val=0
        self.max_limit=0    
        self.min_limit=0
        init = [0, 0, 0, 0]
        self.data = np.array(init, dtype=np.int32)
        #publisher
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        # Subscriber
        rospy.Subscriber('robot_position', Int64MultiArray,self.translation_callback)
        rospy.Subscriber('yaw_axis_data', Int64,self.yaw_callback)
        rospy.Subscriber('depth_data',Int64,self.depth_callback)
    

    def run(self):
        self.limit_decision()
        self.velocity_control(self.robot_data,self.yaw_data,self.depth)
        self.pub_twist_data(self.twist)
      

    #callback関数

    def translation_callback(self,robot_position):
        data=robot_position.data
        self.robot_data=data[3]
        print(self.robot_data)

    def yaw_callback(self,yaw):
        self.yaw_data=yaw.data
        print(self.yaw_data)
  
    def depth_callback(self,depth):        
        last_depth=depth
        self.depth=(depth-self.last_depth)*100
        self.last_depth=last_depth
        print(self.depth)

    def velocity_control(self,top_marker,bottom_marker,depth):
        control_input=(top_marker-bottom_marker)*0.01
        self.twist.linear.x=self.calc_val_velocity(depth)
        self.twist.angular.z=self.calc_val_angle(control_input)
        


    #ロボットの並進方向の決定
    def calc_val_velocity(self,val):
        if  abs(val)<limit :
            val=self.last_val
            return val
        else :    
            if val>0:
                val=-val
            else :
                val=-val  
            return val  
        self.last_val=val

    #ロボットのヨー角の変更
    def calc_val_angle(self,val):
        if  abs(val)<limit :
            val=self.last_val
            return -val
        else :    
            if val>0:
                val=-val
            else :
                val=-val  
            return val  
        self.last_val=val


    #滑らかに動くようにmax,minの値を操作
    def limit_decision(self):
        pass




    #Twistの値をpub
    def pub_twist_data(self,twist):
        self.pub.publish(twist)


def shutdown() :
    pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    twist=Twist()
    twist.angular.x=0
    twist.angular.y=0
    twist.angular.z=0
    twist.linear.x=0
    twist.linear.y=0
    twist.linear.z=0
    pub.publish(twist)


def main():
    calc=Motor_Run()  
    rate=rospy.Rate(3) 
    rospy.on_shutdown(shutdown)       
    while not rospy.is_shutdown(): 
        start = time.time()
        calc.run()
        # 制御周期
        elapsed_time = time.time() - start
        rate.sleep()

    

if __name__=='__main__':
    rospy.init_node('motor_twist')  
    main()
   

