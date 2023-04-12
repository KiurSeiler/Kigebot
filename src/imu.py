#!/usr/bin/env python3

import time
import smbus
import rospy
from sensor_msgs.msg import Imu

class imu_data:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.address = 0x68
        self.power_mgmt_1 = 0x6B
        self.accel_xout_h = 0x3B
        self.gyro_xout_h = 0x43

        # Initialize the MPU6050
        self.bus.write_byte_data(self.address, self.power_mgmt_1, 0)

    def read_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) + low
        return value

    def read_word_2c(self, reg):
        val = self.read_word(reg)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def read_data(self):
        accel_data = {}
        gyro_data = {}

        accel_data['x'] = self.read_word_2c(self.accel_xout_h) / 16384.0
        accel_data['y'] = self.read_word_2c(self.accel_xout_h + 2) / 16384.0
        accel_data['z'] = self.read_word_2c(self.accel_xout_h + 4) / 16384.0

        gyro_data['x'] = self.read_word_2c(self.gyro_xout_h) / 131.0
        gyro_data['y'] = self.read_word_2c(self.gyro_xout_h + 2) / 131.0
        gyro_data['z'] = self.read_word_2c(self.gyro_xout_h + 4) / 131.0

        return accel_data, gyro_data


if __name__ == '__main__':
    rospy.init_node('imu_data', anonymous=True)
    imu_pub = rospy.Publisher('~imu_data', Imu, queue_size=1)

    imu_data = imu_data()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        accel_data, gyro_data = imu_data.read_data()
        print("Accel: ", accel_data)
        print("Gyro: ", gyro_data)

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = accel_data['x']
        imu_msg.linear_acceleration.y = accel_data['y']
        imu_msg.linear_acceleration.z = accel_data['z']

        imu_msg.angular_velocity.x = gyro_data['x']
        imu_msg.angular_velocity.y = gyro_data['y']
        imu_msg.angular_velocity.z = gyro_data['z']

        imu_pub.publish(imu_msg)

        rate.sleep()