#!/usr/bin/env python3

import os
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
import time
import pid

speed = WheelsCmdStamped()

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.pub = rospy.Publisher('/kigebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.odosub = rospy.Subscriber('odometry_publisher',String, self.odometry_callback)
        self.tofsub = rospy.Subscriber('tof_publisher', String, self.tof_callback)
        
        #CALLBACK MUUTUJAD
        
        self.bus = SMBus(1)
        self.ododata = 0
        self.tofdata = 0
        self.piddata = 0
        
        #PID MUUTUJAD
        
        self.previous_left = 0
        self.previous_right = 0
        self.lasterror = 0
        self.lastcorrection = 0
        
    def odometry_callback(self, data):
        self.ododata = data.data
        
    def tof_callback(self, data):
        self.tofdata = data.data
        

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        time.sleep(0.5)
        self.bus.close()
        
    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.tofdata == "wall in progress":
                pass
            else:
                line_sens, read, correction, error= pid.PidClass().pid_run(self.lasterror)
                max_speed = float(rospy.get_param("/maxvel"))
                
                if line_sens == [0,0,0,0,1,1,1,1] or line_sens == [0,0,0,0,0,1,1,1]: #parem 90 kraadi
                    speed.vel_left = max_speed*2
                    speed.vel_right = 0
                    self.pub.publish(speed)
                if line_sens == [1,1,1,1,0,0,0,0] or line_sens == [1,1,1,0,0,0,0,0]: #vasak 90 kraadi
                    speed.vel_left = 0
                    speed.vel_right = max_speed*2
                    self.pub.publish(speed)
                
                if correction < -1 or correction > 1:
                    correction = self.lastcorrection
                else:
                    self.lasterror = error
                    self.lastcorrection = correction
                
                
                speed.vel_left = max_speed - correction
                speed.vel_right = max_speed + correction
                if len(line_sens) == 0:
                    speed.vel_left = self.previous_left
                    speed.vel_right = self.previous_right
                speed.vel_left = max(0.0, min(speed.vel_left, 0.5))
                speed.vel_right = max(0.0, min(speed.vel_right, 0.5))
                self.previous_left = speed.vel_left
                self.previous_right = speed.vel_right
                self.pub.publish(speed)
                
                print("---| P =", rospy.get_param("/p"),
                      "|---| I =", rospy.get_param("/i"),
                      "|---| D =", rospy.get_param("/d"),
                      '|---| Speed =', rospy.get_param("/maxvel"),
                      "|---")
                #print("Odomeetria publisher: ", self.ododata)
                print('correction = ', correction)
                print(read)
            rate.sleep()
            
if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()