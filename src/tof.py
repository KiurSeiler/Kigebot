#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
import numpy as np
from sensor_msgs.msg import Range
import time

speed = WheelsCmdStamped()

class TofPublish(DTROS):
    def __init__(self, node_name):
        super(TofPublish, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.tofpub = rospy.Publisher('tof_publisher', String, queue_size=10)
        self.tof = rospy.Subscriber('/kigebot/front_center_tof_driver_node/range', Range, self.tof_sensor_callback)
        self.pub = rospy.Publisher('/kigebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
    
        self.kaugus = 0
        self.range = 0
        
    def tof_sensor_callback(self,data):
        self.range = data.range
    
    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.kaugus_cm = round(self.range*100, 1)
            while self.kaugus_cm <= 10.0:
                self.kaugus_cm = round(self.range*100, 1)
                speed.vel_right = 0
                speed.vel_left = 0.06
                self.pub.publish(speed)
                #time.sleep(0.5)
                self.sein = 1
                self.tofpub.publish("wall in progress")

            if self.sein == 1:
                speed.vel_right = 0.45
                speed.vel_left = 0.3
                self.pub.publish(speed)
                time.sleep(2.5)
                self.sein = 0
                self.tofpub.publish("no wall")
            rate.sleep()

if __name__ == '__main__':
    node = TofPublish(node_name='tof_publisher')
    node.run()
    rospy.spin()