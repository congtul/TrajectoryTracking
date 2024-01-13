#! /usr/bin/env python

import serial
import math
import rospy
from custom_msg.msg import imu_data

def main():
    yaw_tmp_3 = 0.00
    port = rospy.get_param('~port', '/dev/ttyUSB1')
    baudrate = rospy.get_param('~baud', '230400')
    ser = serial.Serial(port, baudrate)
    imu_pub = rospy.Publisher('yaw_imu', imu_data, queue_size=10)
    yaw_imu_msg = imu_data()
    while not rospy.is_shutdown():
        raw_yaw = ser.readline()
        print(raw_yaw)
        yaw_tmp_1 = raw_yaw.replace('  ', ' ')
        yaw_tmp_2 = yaw_tmp_1.split(' ')

        if(len(yaw_tmp_2) == 10):
            # rospy.loginfo(yaw_2[2])
            yaw_tmp_3 = float(yaw_tmp_2[2])
            # rospy.loginfo(yaw_imu)
        elif(len(yaw_tmp_2) == 11):
            yaw_tmp_3 = float(yaw_tmp_2[3])
            # rospy.logsinfo(yaw_imu)
        else:
            print("NOT ENOUGH ELEMENTS")

        yaw_tmp_3 = yaw_tmp_3/1000.0
        yaw_imu_msg.yaw = yaw_tmp_3
        rospy.loginfo(yaw_imu_msg)
        imu_pub.publish(yaw_imu_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('imu_node')
        rate = rospy.Rate(20)
        main()
    except rospy.ROSInterruptException:
        pass
