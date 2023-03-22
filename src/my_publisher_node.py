#!/usr/bin/env python3

import os
import numpy as np
import rospy
import smbus2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
import time

speed = WheelsCmdStamped()

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.pub = rospy.Publisher('/kigebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.odosub = rospy.Subscriber('odometry_publisher',String, self.odometry_callback)
        self.tofsub = rospy.Subscriber('tof_publisher', String, self.tof_callback)
        #self.tof = rospy.Subscriber('/kigebot/front_center_tof_driver_node/range', Range, self.tof_callback)
        self.pidsub = rospy.Subscriber('pid_publisher', String, self.pid_callback)
        
        # MUUTUJAD
        
        self.bus = SMBus(1)
        self.ododata = 0
        self.tofdata = 0
        self.piddata = 0
        
        # PID MUUTUJAD
        
        self.sens_middle = 4.5
        self.delta_time = 1/20
        self.last_error = 0
        self.integral = 0
        self.prev_integral = 0
        self.previous_left = 0
        self.previous_right = 0
        
    def odometry_callback(self, data):
        self.ododata = data.data
        
    def tof_callback(self, data):
        self.tofdata = data.data
        
    def pid_callback(self, data):
        self.piddata = data.data
    
    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        time.sleep(0.5)
        self.bus.close()
        
    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
                        
            #self.odolist = []
            #self.odolist = self.odo
             
            if self.tofdata == "wall in progress":
                pass
            elif self.tofdata == "no wall":
                read = self.bus.read_byte_data(62,17)
                read = bin(read)[2:].zfill(8)
                
                Kp = rospy.get_param("/p") # 0.25
                Ki = rospy.get_param("/i") # 0.03
                Kd = rospy.get_param("/d") # 0.5
                    
                max_kiirus = 0   
                näidik = []
                for indx, nr in enumerate(read):
                    if nr == "1":
                        näidik.append(indx + 1)
                error = self.sens_middle - np.average(näidik)
                print("error =", error)
                integral = self.prev_integral + error*self.delta_time
                integral = max(min(integral,2), -2)
                derivative = (error - self.last_error)/self.delta_time
                correction = Kp * error + Ki * integral + Kd * derivative
                print("correction = ", correction)
                speed.vel_left = max_kiirus - correction
                speed.vel_right = max_kiirus + correction
                if len(näidik) == 0:
                    speed.vel_left = self.previous_left
                    speed.vel_right = self.previous_right
                speed.vel_left = max(0.02, min(speed.vel_left, max_kiirus))
                speed.vel_right = max(0.02, min(speed.vel_right, max_kiirus))
                self.previous_left = speed.vel_left
                self.previous_right = speed.vel_right
                self.pub.publish(speed)
                self.last_error = error
                rate.sleep()
                
                print("---| P =", Kp, "|---| I =", Ki, "|---| D =", Kd, "|---")
                print("Odomeetria publisher: ", self.ododata)
                #print(self.piddata)
            
if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()