#! /usr/bin/env python

import serial
import rospy
import utm
from custom_msg.msg import gps_data

def main():
    global newdata
    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baudrate = rospy.get_param('~baud', '115200')
    ser = serial.Serial(port, baudrate)
    gps_pub = rospy.Publisher('gps_data',gps_data, queue_size=10)
    while not rospy.is_shutdown():
        raw_rmc = ser.readline()
        rmc = raw_rmc.split(',')
        if "$GPRMC" or "$GNRMC" in rmc:           
            # if (rmc[2] == 'A' and len(rmc) == 12):
            if (rmc[2] == 'A'):
                # "4832.1234," it would extract "48" as the degrees
                # latitude takes 2 fixed digits
                latitude_degrees = int(rmc[3][:2])
                # "4832.1234," it would extract "32.1234" as the minutes.
                latitude_minutes = float(rmc[3][2:])
                # longitude takes 3 fixed digits
                longitude_degrees = int(rmc[5][:3])
                longitude_minutes = float(rmc[5][3:])
                longitude = longitude_degrees + (longitude_minutes / 60)
                latitude = latitude_degrees + (latitude_minutes / 60)
                if rmc[4] == 'S' and rmc[6] == 'W':
                    latitude *= -1
                if rmc[6] == 'W':
                    longitude *= -1
                
                utm_coord = utm.from_latlon(latitude, longitude)
                easting = utm_coord[0]
                northing = utm_coord[1]
                
                gps_msg = gps_data()
                gps_msg.easting = easting
                gps_msg.northing = northing
                print("easting: " + str(gps_msg.easting))
                print("northing: " + str(gps_msg.northing))
                gps_msg.newdata_available = 1
                gps_pub.publish(gps_msg)
                
                print(gps_msg.easting)
                print(gps_msg.northing)
            else:
                print("Navigation receiver warning")
                
            
if __name__ == '__main__':
    try:
        rospy.init_node('gps_node')
        rate = rospy.Rate(20)
        main()
    except rospy.ROSInterruptException:
        pass

