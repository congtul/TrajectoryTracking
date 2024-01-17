#include <ros/ros.h> 
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "sensor_msgs/NavSatFix.h"
#include <serial/serial.h>
#include "std_msgs/Float64.h"
#include "my_msgs/GPS.h"
serial::Serial sp;
std::string data;
std::vector<std::string> subdata;
my_msgs::GPS msg;
unsigned char data_len, checksum1 = 0, checksum2 = 0;
double latitude, longitude, speed;
char mode;
/**
 * $GNGGA

Format: $GNGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,M,<10>,M,< 11>,<12>*xx<CR><LF> 
E.g: $GNGGA,072446.00,3130.5226316,N,12024.0937010,E,4,27,0.5,31.924,M,0.000,M,2.0,*44 Field explanation:

        <0> $GNGGA
        <1> UTC time, the format is hhmmss.sss
        <2> Latitude, the format is ddmm.mmmmmmm
        <3> Latitude hemisphere, N or S (north latitude or south latitude)
        <4> Longitude, the format is dddmm.mmmmmmm
        <5> Longitude hemisphere, E or W (east longitude or west longitude)
        <6> GNSS positioning status: 0 not positioned, 1 single point positioning, 2 differential GPS fixed solution, 4 fixed solution, 5 floating point solution
        <7> Number of satellites used
        <8> HDOP level precision factor
        <9> Altitude
        <10> The height of the earth ellipsoid relative to the geoid
        <11> Differential time
        <12> Differential reference base station label
        * Statement end marker
        xx XOR check value of all bytes starting from $ to *
        <CR> Carriage return, end tag
        <LF> line feed, end tag
*/

/**
 * $GNRMC

Format: $GNRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,< 12>*xx<CR><LF> 
E.g: $GNRMC,072446.00,A,3130.5226316,N,12024.0937010,E,0.01,0.00,040620,0.0,E,D*3D Field explanation:

        <0> $GNRMC
        <1> UTC time, the format is hhmmss.sss
        <2> Positioning status, A=effective positioning, V=invalid positioning
        <3> Latitude, the format is ddmm.mmmmmmm
        <4> Latitude hemisphere, N or S (north latitude or south latitude)
        <5> Longitude, the format is dddmm.mmmmmmm
        <6> Longitude hemisphere, E or W (east longitude or west longitude)
        <7> Ground speed
        <8> Ground heading (take true north as the reference datum)
        <9> UTC date, the format is ddmmyy (day, month, year)
        <10> Magnetic declination (000.0~180.0 degrees)
        <11> Magnetic declination direction, E (east) or W (west)
        <12> Mode indication (A=autonomous positioning, D=differential, E=estimation, N=invalid data)
        * Statement end marker
        XX XOR check value of all bytes starting from $ to *
        <CR> Carriage return, end tag
        <LF> line feed, end tag
*/
double CalLat2Deg(double Lat)
{
	static uint8_t Deg=0;
static double Min=0,  Result=0;
	Deg= Lat/100;
	Min=Lat-Deg*100;
	Result=Deg+Min/60;
return  Result; 
}
	
double CalLong2Deg(double Long)
{
	static uint16_t Deg=0;
	static double Min=0,  Result=0;
	Deg= Long/100;
	Min=Long-Deg*100;
	Result=Deg+Min/60;
return  Result; 
}
int main(int argc, char **argv)
{
	// // string port("/dev/ttyUSB_GPS"); //achieve name of port
	// string port("/dev/ttyACM0"); //achieve name of port
	
	ros::init(argc, argv, "GPS_node");
	ros::NodeHandle n;
	ROS_INFO("[read_GPS] Start Init GPS Node!!!");

	// serial::Timeout to = serial::Timeout::simpleTimeout(200);
	// // Set the name of the serial port to be opened
	// sp.setPort(port);
	// // Set the baud rate of serial communication
	// sp.setBaudrate(115200);
	// // Serial port set timeout
	// sp.setTimeout(to);
	ros::Publisher GPS_read = n.advertise<my_msgs::GPS>("GPS_node", 1000);
	// // ros::Timer timer1 = n.createTimer(ros::Duration(0.1), Timer1_Callback);
	// sp.open();
	// ros::Rate loop_rate(1/0.1);
	
	// if(sp.isOpen())
    // 	cout << " Yes." << endl;
  	// else
	// 	cout << " No." << endl;	
	// ros::Duration(1).sleep(); // waiting for serial is ready
    ros::Rate loop_rate(10);
	while (ros::ok())
	{
        // --------------------DEBUG-------------------//
        msg.lat = 1;
        msg.lng = 2;
        msg.speed = 3;
        GPS_read.publish(msg);
        loop_rate.sleep();
        //--------------------DEBUG-----------------//
	// 	if(sp.isOpen())
	// 	{
	// 		//ROS_INFO("GPS_read");
	// 		try
	// 		{
	// 			data = sp.readline();
	// 			checksum1 = 0;
	// 			checksum2 = 0;
	// 			data_len = data.length();
	// 			checksum1 = (data[data_len-4]-0x30)<<4 | (data[data_len-3]-0x30);   //achieve CRC from frame
	// 			for(int i=1;i<data_len-5;i++) checksum2 ^= data[i];  //calculate CRC of frame
	// 			//cout<<checksum1<<endl;
	// 			//cout<<checksum2<<endl;
	// 			boost::split(subdata, data, boost::is_any_of(",")); 	
	// 		}
	// 		//serial::IOException
	// 		catch(serial::IOException const& err)
	// 		{
	// 			cout<<"Cannot navigate..."<<endl;
	// 		}

	// 		if(subdata[0] == "$GNGGA" && subdata[2] != "" && checksum1 == checksum2)
	// 		{
			
	// 			latitude = std::strtod(subdata[2].c_str(),0);
	// 			longitude = std::strtod(subdata[4].c_str(),0);
	// 			mode = subdata[6][0];  //0 not positioned, 1 single point positioning, 2 differential GPS fixed solution, 4 fixed solution, 5 floating point solution
	// 		}
	// 		else if(subdata[0] == "$GNRMC" && subdata[2] != "" && checksum1 == checksum2) 
	// 		{
	// 			latitude = std::strtod(subdata[3].c_str(),0);
	// 			longitude = std::strtod(subdata[5].c_str(),0);
	// 			speed = std::strtod(subdata[7].c_str(),0);
	// 			mode = subdata[12][0]; // A=autonomous positioning, D=differential, E=estimation, N=invalid data
	// 		}
	// 	}

	// 	// public msg
	// 	msg.lat = CalLat2Deg(latitude);
	// 	msg.lng = CalLong2Deg(longitude);
	// 	msg.speed = speed;
	// 	ROS_INFO("GPS_read: lat=%f, lng=%f, speed=%f, mode=%c", msg.lat*1000, msg.lng*1000, msg.speed, mode);
	// 	//cout<<""<<msg.lat<<",";
	// 	//cout<<msg.lng<<endl;
		
	// 	GPS_read.publish(msg);
	// 	// ros::spinOnce();
	// 	// loop_rate.sleep();
	// 	//ros::Duration(0.2).sleep();
	}	
	return 0;
}
