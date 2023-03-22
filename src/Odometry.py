#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
import numpy as np

class OdometryNode(DTROS):
    def __init__(self, node_name):
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher('odometry_publisher',String, queue_size=10)
        self.rwheel = rospy.Subscriber('/kigebot/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.rightwheel)
        self.lwheel = rospy.Subscriber('/kigebot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.leftwheel)
        self.seqLeft = rospy.Subscriber('/kigebot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.time_leftwheel)
        self.seqRight = rospy.Subscriber('/kigebot/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.time_rightwheel)

        #-------------------VARIABLES-------------------
        
        self.right = 0
        self.left = 0
        self.timeL = 0
        self.timeR = 0
        self.ticks_left = 0
        self.ticks_right = 0
        self.prev_tick_left = 0
        self.prev_tick_right = 0
        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0
        self.delta_ticks_left = 0
        self.delta_ticks_right = 0
        self.baseline_wheel2wheel = 0.1                                         # rataste keskel vahe, meetrites
        
    #--------------------CALLBACK FUNCTIONS-------------------    
        
    def rightwheel(self, data):
        self.right = data.data
    
    def leftwheel(self, data):
        self.left = data.data

    def time_leftwheel(self, data):
        self.timeL = data.header.seq
    
    def time_rightwheel(self, data):
        self.timeR = data.header.seq 
        
    def Odometry_func(self):
        N_tot = 135                                                             # total number of ticks per revolution
        R = 0.065                                                               # ratta diameeter meetrites
        alpha = 2 * np.pi / N_tot                                               # wheel rotation per tick in radians
        
        self.ticks_right = self.right
        self.ticks_left = self.left
        
        self.delta_ticks_left = self.ticks_left-self.prev_tick_left             # delta ticks of left wheel
        self.delta_ticks_right = self.ticks_right-self.prev_tick_right          # delta ticks of right wheel
        
        self.rotation_wheel_left = alpha * self.delta_ticks_left                # kogu vasaku ratta põõrded radiaanides
        self.rotation_wheel_right = alpha * self.delta_ticks_right              # kogu parema ratta põõrded radiaanides
        
        d_left = R * self.rotation_wheel_left
        d_right = R * self.rotation_wheel_right  
        wtravel = round(((d_left + d_right)*100)/2, 1)                          # mõlema ratta distantsi keskmine sentimeetrites ja ümardatud
    
        Delta_Theta = np.rad2deg((d_right-d_left)/self.baseline_wheel2wheel)    # kogu roboti pööramine kraadides
        
        self.prev_tick_left = self.ticks_left                                   # rataste jooskmis/kestvus aeg
        self.prev_tick_right = self.ticks_right
        
        odolist = f"{self.ticks_right}, {self.ticks_left}, {wtravel}, {Delta_Theta}"
            
        return odolist                                                          # tagastab odomeetria info massiivina
        #return "function return"
        
    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            odom = self.Odometry_func()
            #test_string = "Test publish string"
            #rospy.loginfo("Publishing message: ", test_string)
            self.pub.publish(odom)
            rate.sleep()

if __name__ == '__main__':
    node = OdometryNode(node_name='OdometryNode') #node_name='Odometry'
    node.run()
    rospy.spin()