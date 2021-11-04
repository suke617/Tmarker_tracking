#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import sqrt, cos, sin
import rospy
import numpy as np
import time 
from geometry_msgs.msg import Twist
import rosparam
from std_msgs.msg import Int64MultiArray, Int64, String



"""
@author 吉田圭佑
color_tracking_nodeのpubを受け取りモータの移動量を決定するプログラム

"""

#Tマーカが横方向に1px動いたときの操作量
WIDE=0.1
#Tマーカが奥行き方向に1px動いたときの操作量
HIGHT=0.1
#誤差の許容範囲
limit=0.1
     


class Motor_Run():  
    def __init__(self):
        self.twist=Twist()
        self.robot_data1=0
        self.yaw_data=0
        init = [0, 0, 0, 0]
        self.data = np.array(init, dtype=np.int32)
        #publisher
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        # Subscriber
        rospy.Subscriber('robot_position', Int64MultiArray,self.translation_callback)
        rospy.Subscriber('yaw_axis_data', Int64,self.yaw_callback)
        #rospy.Subscriber('',Int64MultiArray,self.depth_callback)
    

    def run(self):
        self.yaw_control(self.robot_data1,self.yaw_data)
        self.pub_twist_data(self.twist)
      

    def translation_callback(self,robot_position):
        data=robot_position.data
        self.robot_data1=data[3]
        print(self.robot_data1)

    def yaw_callback(self,yaw):
        self.yaw_data=yaw.data
        print(self.yaw_data)
  
    
    #def depth_callback(self,depth):
        #self.depth=depth


    def yaw_control(self,top_marker,bottom_marker):
        control_input=top_marker-bottom_marker
        self.twist.linear.x=0.01
        #self.twist.linear.x=self.calc_val_velocity(yaw_control_input)
        self.twist.angular.z=control_input*0.01
        #self.calc_val_angle(control_input)


    #ロボットの並進方向の決定
    def calc_val_velocity(self,val):
         new_val=val/HIGHT
         if abs(new_val)<limit :
           val=val
         else :
            val=new_val
         return val 

    #ロボットのヨー角の変更
    def calc_val_angle(self,val):
        if  abs(val)<limit :
            val=0
            return val
        else :    
            if val<0:
                val=val*WIDE
            else :
                val=-val*WIDE  
            return val  


    #Twistの値をpub
    def pub_twist_data(self,twist):
        self.pub.publish(twist)




#keigan_motorの停止
def shutdown():
    twist=Twist()
    twist.linear.x=0
    twist.linear.y=0
    twist.linear.z=0
    twist.angular.x=0
    twist.angular.y=0
    twist.angular.z=0



    

if __name__=='__main__':
    rospy.init_node('motor_twist')  
    calc=Motor_Run()  
    rate=rospy.Rate(10)        
    #ctrl+cしたときに実行
    rospy.on_shutdown(shutdown)  
    while not rospy.is_shutdown(): 
        start = time.time()
        calc.run()
        # 制御周期
        elapsed_time = time.time() - start
        rate.sleep()


