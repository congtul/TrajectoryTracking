// 1 timer serial with STM
// 1 timer for controller
// 1 timer for UI
#include <ros/ros.h>
#include <iostream>
#include <boost/algorithm/string.hpp>

#include "std_msgs/String.h" //Ros defined String data type
#include "sensor_msgs/NavSatFix.h"
#include <serial/serial.h>
#include "std_msgs/Float64.h"
#include "my_msgs/GPS.h"
#include "my_msgs/IMU.h"

serial::Serial sp;
std::string data;
std::vector<std::string> subdata;
double yaw;
double lat,lng;
ros::Publisher controller;


//void Callback_GUI_config(void);
void Callback_IMU(const my_msgs::IMU& msg);
void Callback_GPS(const my_msgs::GPS& msg);
void Callback_serial_STM(const ros::TimerEvent&);
void Callback_controller(const ros::TimerEvent&);
void Callback_UI(const ros::TimerEvent&);
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Controller_node");
	ros::NodeHandle n;
	ROS_INFO("[Main] Start Init Controller Node!!!");
    // Set up multi-threaded spinner with 4 threads
    ros::MultiThreadedSpinner spinner(4);
    //std::string port("/dev/ttyUSB_IMU"); //achieve name of port	
    // serial::Timeout to = serial::Timeout::simpleTimeout(20);
    // // Set the name of the serial port to be opened
	// sp.setPort(port);
	// 	// Set the baud rate of serial communication
	// sp.setBaudrate(115200*2);
	// 	// Serial port set timeout
	// sp.setTimeout(to);
    // sp.open();
    // if(sp.isOpen())
    // 	std::cout << " Yes." << std::endl;
  	// else
   	//  	std::cout << " No." << std::endl;
	// ros::Duration(1).sleep(); // waiting for serial is ready
    controller = n.advertise<my_msgs::IMU>("Controller_node", 1000);
    ros::Subscriber sub_IMU = n.subscribe("IMU_node", 1000, Callback_IMU);
    ros::Subscriber sub_GPS = n.subscribe("GPS_node", 1000, Callback_GPS);
    //ros::Subscriber sub_GUI = n.subscribe("GUI_config", 1000, Callback_GUI_config);
    ros::Timer timer1 = n.createTimer(ros::Duration((double)0.1), Callback_controller); // 100ms for 1 controll cycle
    ros::Timer timer2 = n.createTimer(ros::Duration((double)0.015), Callback_serial_STM); // Read UART from MCU every 15ms
    ros::Timer timer3 = n.createTimer(ros::Duration((double)0.015), Callback_UI); // publish data to topic for GUI every 15ms
    spinner.spin();
    while(ros::ok())
    {

    }
    std::cout<<"............Ending............"<<std::endl;  
    return 0;
}
// void Callback_GUI_config(void)
// {

// }
void Callback_IMU(const my_msgs::IMU& msg)
{
    yaw = msg.yaw;
}
void Callback_GPS(const my_msgs::GPS& msg)
{
    lat = msg.lat;
    lng = msg.lng;
}
void Callback_serial_STM(const ros::TimerEvent&)
{
    ROS_INFO("STM");
}
void Callback_controller(const ros::TimerEvent&)
{
    ROS_INFO("Controller");
    my_msgs::IMU msg;
    msg.roll = 2;
    msg.pitch = 3;
    msg.yaw = yaw;
    controller.publish(msg);
}
void Callback_UI(const ros::TimerEvent&)
{
    ROS_INFO("GUI");
}
