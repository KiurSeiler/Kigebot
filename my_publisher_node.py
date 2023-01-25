#!/usr/bin/env python3

import os
import rospy
import smbus2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

speed = WheelsCmdStamped()

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('/kigebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
    
    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()
    
    def run(self):
        #bus = SMBus(1)
        # publish message every 1 second
        rate = rospy.Rate(18) # 1Hz
        viimased_kümme = []
        turn = 2
        while not rospy.is_shutdown():
            bus = SMBus(1)
            read = bus.read_byte_data(62,17)
            if len(viimased_kümme) <= 5:
                viimased_kümme.append(read)
            else:
                viimased_kümme.pop(0)
                viimased_kümme.append(read)
            
            B0 = 1 
            B1 = 2
            B2 = 4
            B3 = 8
            B4 = 16
            B5 = 32
            B6 = 64
            B7 = 128
            
            keskel = (B3, B4,
                      B3+B4)
            
            parem = (B0, B1, B2,
                     B1+B2, B0+B1, B3+B2,
                     B0+B1+B2)
            
            vasak = (B7, B6, B5,
                     B7+B6, B6+B5, B4+B5,
                     B7+B6+B5)
            
            # 1-4 pooramiskiirus
            vasak1 = B5
            vasak2 = B6
            vasak4 = B7
                        
            parem1 = B2
            parem2 = B1
            parem4 = B0

            ###################
            #kiirus on väärtus millega robot liigub kui joon ei ole keskel
            kiirus = 0.1
            #vahe on väärtus mis on kahe mootori vaheline kiiruseerinevus pööramisel
            vahe = 0.08
            
            if read in keskel:
                speed.vel_left = 0.5
                speed.vel_right = 0.5
                turn = 2
            elif read in parem:
                if read == parem1:
                    speed.vel_right = kiirus
                    speed.vel_left = kiirus+vahe
                elif read == parem2:
                    speed.vel_right = kiirus
                    speed.vel_left = kiirus + 2*vahe
                elif read == parem4:
                    speed.vel_right = kiirus+0.02
                    speed.vel_left = kiirus+3*vahe
                turn = 1
            elif read in vasak:
                if read == vasak1:
                    speed.vel_right = kiirus + vahe
                    speed.vel_left = kiirus
                elif read == vasak2:
                    speed.vel_right = kiirus + 2*vahe
                    speed.vel_left = kiirus
                elif read == vasak4:
                    speed.vel_right = kiirus + 3*vahe
                    speed.vel_left = kiirus+0.02
                turn = 0
            elif read == 0:
                if turn == 0 or turn == 1:
                    if turn == 1:
                        speed.vel_right = kiirus - vahe*1.5
                        speed.vel_left = kiirus
                    elif turn == 0:
                        speed.vel_right = kiirus
                        speed.vel_left = kiirus - vahe*1.5  
                else:
                    continue
            else:
                speed.vel_right = 0
                speed.vel_left = 0
            self.pub.publish(speed)
            rate.sleep()
            bus.close()
            print(read)
            
if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    node.run()
    rospy.spin()