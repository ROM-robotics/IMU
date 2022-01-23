#!/usr/bin/env python3

import time
import smbus
import struct
import rospy
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_about_axis

# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_PIN_CFG  = 0x37
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

ADDR = None
bus = None
IMU_FRAME = None

# read_word and read_word_2c from http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def publish_temp(timer_event):
    temp_msg = Temperature()
    temp_msg.header.frame_id = IMU_FRAME
    temp_msg.temperature = read_word_2c(TEMP_H)/340.0 + 36.53
    temp_msg.header.stamp = rospy.Time.now()
    #temp_pub.publish(temp_msg)


def publish_imu(timer_event):
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    # Read the acceleration vals
    accel_x = read_word_2c(ACCEL_XOUT_H) / 2048.0
    accel_y = read_word_2c(ACCEL_YOUT_H) / 2048.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 2048.0
    
    # Calculate a quaternion representing the orientation
    '''accel = accel_x, accel_y, accel_z
    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    orientation = quaternion_about_axis(angle, axis)'''

    # Read the gyro vals
    gyro_x = read_word_2c(GYRO_XOUT_H) / 16.4
    gyro_y = read_word_2c(GYRO_YOUT_H) / 16.4
    gyro_z = read_word_2c(GYRO_ZOUT_H) / 16.4
    
    # Load up the IMU message
    '''o = imu_msg.orientation
    o.x, o.y, o.z, o.w = orientation'''

    imu_msg.linear_acceleration.x = accel_x*9.8
    imu_msg.linear_acceleration.y = accel_y*9.8
    imu_msg.linear_acceleration.z = accel_z*9.8

    imu_msg.angular_velocity.x = gyro_x*0.0174
    imu_msg.angular_velocity.y = gyro_y*0.0174
    imu_msg.angular_velocity.z = gyro_z*0.0174

    imu_msg.header.stamp = rospy.Time.now()

    imu_pub.publish(imu_msg)

def IMU_configure():
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0X18)
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0X18)

temp_pub = None
imu_pub = None

if __name__ == '__main__':
    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    IMU_FRAME = rospy.get_param('~imu_frame', 'imu')

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)
    time.sleep(0.5)
    IMU_configure()
    
    #temp_pub = rospy.Publisher('temperature', Temperature,queue_size=10)
    imu_pub = rospy.Publisher('imu/data_raw', Imu,queue_size=10)
    imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
    #temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)
    rospy.spin()
