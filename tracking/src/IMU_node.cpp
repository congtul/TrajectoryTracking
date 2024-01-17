#include <ros/ros.h>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "my_msgs/IMU.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <serial/serial.h>
#include "std_msgs/Float64.h"
std::string data;
std::vector<std::string> subdata;
my_msgs::IMU msg;
serial::Serial sp;
double roll, pitch, yaw, pre_yaw;
int main(int argc, char **argv)
{
    // std::string port("/dev/ttyUSB0");
    ros::init(argc, argv, "IMU_node");
    ros::NodeHandle n;
    ROS_INFO("[read_IMU] Start Init IMU Node!!!");
    // serial::Timeout to = serial::Timeout::simpleTimeout(20);
    //      // Set the name of the serial port to be opened
	// sp.setPort(port);
	// 	// Set the baud rate of serial communication
	// sp.setBaudrate(115200*2);
	// 	// Serial port set timeout
	// sp.setTimeout(to);
    ros::Publisher IMU_read = n.advertise<my_msgs::IMU>("IMU_node", 1000);
    // ROS_INFO("Starting open IMU port");
    // sp.open();
    // if(sp.isOpen())
    // 	std::cout << " IMU connected." << std::endl;
  	// else
   	//  	std::cout << " Waiting for IMU connect...." << std::endl;
    // ros::Duration(1).sleep(); // waiting for serial is ready
    ros::Rate loop_rate(10);
 	while (ros::ok())
    {
        // --------------------DEBUG-------------------//
        msg.roll = 1;
        msg.pitch = 2;
        msg.yaw = 3;
        IMU_read.publish(msg);
        loop_rate.sleep();
        //--------------------DEBUG-----------------//
        // if(sp.isOpen())
		// {
		// 	static int flag_yaw = 1, cnt_yaw = 0;	
		// 	try
		// 	{
		// 		data = sp.readline(100, "\r\n");
		// 		if(data.size() < 70) {
		// 			ROS_ERROR("IMU Read: data.size() < 70");
		// 			continue;
		// 		}
		// 		std::cout<<"sp.readline():"<< data << std::endl;		
		// 		// std::cout<<"sp.readline():"<< data[0] << std::endl;
		// 	}
		// 	catch(boost::exception const& err)
		// 	{
		// 		std::cout<<"Cannot navigate..." << std::endl;
		// 	}
		// 	/********** Data Format ****************/
		// 	/* |0x0A   roll   pitch   yaw    wx     wy     wz     ax     ay     az    0x0D| */
		// 	/* |1byte 7bytes 7bytes 7bytes 7bytes 7bytes 7bytes 6bytes 6bytes 6bytes 1byte| */
		// 	// NOTE: data = sp.readline() is not contain "Start Byte"-0x0A
		// 	// Get string of roll, pitch, yaw
		// 	std::string roll_str = data.substr(0,7); // roll_string is start at index 0, size = 7 bytes
		// 	subdata.push_back(roll_str); 
		// 	std::string pitch_str = data.substr(8,7); // roll_string is start at index 8, size = 7 bytes 
		// 	subdata.push_back(pitch_str);
		// 	std::string yaw_str = data.substr(16,7); // roll_string is start at index 16, size = 7 bytes
		// 	subdata.push_back(yaw_str); 
			
		// 	roll = (std::strtod(subdata[0].c_str(),0))*0.001;
		// 	pitch = (std::strtod(subdata[1].c_str(),0))*0.001;
		// 	yaw = (std::strtod(subdata[2].c_str(),0))*0.001;

		// 	subdata.clear(); // Clear subdata

		// 	// Convert to ...	
		// 	msg.roll = roll;
		// 	msg.pitch = pitch;
		// 	yaw = yaw;
		// 	// yaw = -180-->180 deg
		// 	if(flag_yaw == 1){cnt_yaw = 0;flag_yaw = 0;}
		// 	if(yaw>=150 && pre_yaw<-150){cnt_yaw--;}
		// 	else if(yaw<-150 && pre_yaw>=150){cnt_yaw++;}
		// 	// msg.roll = yaw_dot;
		// 	// msg.pitch = yaw_dot_dot;
		// 	msg.yaw = yaw + cnt_yaw*360;

		// 	// std::cout<<"roll="<<msg.roll<<", pitch="<<msg.pitch<<",yaw="<<msg.yaw<<std::endl;
		// 	// std::cout<<Convertheading(msg.yaw, mode_IMU);
		// 	//std::cout<<"flag_yaw="<<flag_yaw<<std::endl;
		// 	//std::cout<<"cnt_yaw="<<cnt_yaw<<std::endl;
		// 	//std::cout<<"pre_yaw="<<pre_yaw<<std::endl;

		// 	// std::cout << msg.yaw;
		// 	// std::cout<<";";
		// 	// std::cout<<std::endl;
			

		// 	pre_yaw = yaw;	

		// 	// public msg
		// 	// IMU_read.publish(msg);
		// 	// ros::spinOnce();
		// 	// ros::Duration(0.04).sleep();
		// 	// loop_rate.sleep();	
		// } 
		// // ROS_INFO("callback_timer END");


		// // public msg
		// IMU_read.publish(msg);
		// // ros::spinOnce();
		// // ros::spin();
		// // ros::Duration(0.01).sleep();
		// // loop_rate.sleep();
	}
	//ros::Subscriber sub = n.subscribe("GPS", 1000,chatterCallback);
	//ros::spin();
	return 0;
}	