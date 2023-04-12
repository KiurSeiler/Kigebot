#!/usr/bin/env python3

import os
import rospy
import smbus2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
import numpy as np
from smbus2 import SMBus
import my_publisher_node as mpn

class PidClass():
    def __init__(self):
        self.bus = SMBus(1)
        self.sens_middle = 4.5
        self.delta_time = 1/15
        self.error = 0
        self.prev_integral = 0
        self.integral = 0
        self.hard_left = 0
        self.hard_right = 0

    def pid_run(self, last_error):
        # rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            read = self.bus.read_byte_data(62,17)
            read = bin(read)[2:].zfill(8)
            
            Kp = float(rospy.get_param("/p")) # 0.25
            Ki = float(rospy.get_param("/i")) # 0.03
            Kd = float(rospy.get_param("/d")) # 0.5
            
            line_sens = []
            
            for indx, nr in enumerate(read):
                if nr == "1":
                    line_sens.append(indx + 1)
                    
            if len(line_sens) > 0:
                self.error = self.sens_middle - np.average(line_sens)
            else:
                self.error = 0
            
            print(last_error)
            
            #error = sens_middle - np.average(line_sens)
            self.integral = self.integral + (self.error + last_error)*self.delta_time/2
            self.integral = min(max(self.integral, -2), 2)
            derivative = (self.error - last_error)/self.delta_time
            correction = Kp * self.error + Ki * self.integral + Kd * derivative
            
            # rate.sleep()
            return line_sens, read, correction, self.error