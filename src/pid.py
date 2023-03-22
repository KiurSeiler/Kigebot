#!/usr/bin/env python3

import os
import rospy
import smbus2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
import numpy as np
from smbus2 import SMBus

class PidPublish(DTROS):
    def __init__(self, node_name):
        super(PidPublish, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher('pid_publisher', String, queue_size=10)
    
    #----------PID MUUTUJAD----------------
        
        self.bus = SMBus(1)
        self.sens_middle = 4.5
        self.delta_time = 1/20
        self.last_error = 0
        self.integral = 0
        self.prev_integral = 0
        self.previous_left = 0
        self.previous_right = 0
    
    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            read = self.bus.read_byte_data(62,17)
            read = bin(read)[2:].zfill(8)
            
            Kp = rospy.get_param("/p") # 0.25
            Ki = rospy.get_param("/i") # 0.03
            Kd = rospy.get_param("/d") # 0.5
             
            line_sens = []
            for indx, nr in enumerate(read):
                if nr == "1":
                    line_sens.append(indx + 1)
            error = self.sens_middle - np.average(line_sens)
            print("error =", error)
            integral = self.prev_integral + error*self.delta_time
            integral = max(min(integral,2), -2)
            derivative = (error - self.last_error)/self.delta_time
            correction = Kp * error + Ki * integral + Kd * derivative
            # speed.vel_left = max_speed - correction
            # speed.vel_right = max_speed + correction
            # if len(line_sens) == 0:
            #     speed.vel_left = self.previous_left
            #     speed.vel_right = self.previous_right
            # speed.vel_left = max(0.02, min(speed.vel_left, max_speed))
            # speed.vel_right = max(0.02, min(speed.vel_right, max_speed))
            # self.previous_left = speed.vel_left
            # self.previous_right = speed.vel_right
            # self.pub.publish(speed)
            
            print("correction = ", correction)
            print("---| P =", Kp, "|---| I =", Ki, "|---| D =", Kd, "|---")
            self.pub.publish(correction)
            rate.sleep()

if __name__ == '__main__':
    node = PidPublish(node_name='pid_publisher')
    node.run()
    rospy.spin()