
#include <ros/ros.h> // header file containing ROS
//#include <boost/asio.hpp> // Include boost library function

//#include <boost/algorithm/string.hpp>
//#include <boost/lexical_cast.hpp>
//#include <boost/bind.hpp>
// #include <stdio.h>
#include <stdlib.h>
#include "stdio.h"


#include "std_msgs/String.h" //Ros defined String data type
#include <serial/serial.h>
#include <serial/v8stdint.h>
#include <iostream>
#include <string>
#include <fstream>
#include <serial/serial.h>
#include <boost/algorithm/string.hpp>

#include "ugv_1000/defines.hpp"
#include "ugv_msgs/toUGV.h"
#include "ugv_msgs/toGUI.h"
#include "ugv_1000/IMU_msg.h"
#include "ugv_1000/GPS_msg.h"
#include "ugv_1000/PWM.hpp"
#include "ugv_1000/Control_Heading.hpp"
#include "ugv_1000/GPS.hpp"
#include "ugv_1000/Bspline.hpp"
#include "ugv_1000/SBG.hpp"
// #include "ugv_1000/Can.hpp"
#include "ugv_1000/trajectory_path.h"

// MPC lib
#include "osqp.h"
#include "coder_array.h"
// #include "matrix.h"
#include "linear_mpc.h"

// Stanley lib
#include "stanley/stanley.h"

#include <ctime>
#include <sys/time.h>
#include <chrono>
#include "ref_path.h"

#define USE_SPEED_ENCODER
// #define USE_SPEED_GPS
#define SPEED_GPS_INDEX_FILTER_MAX		10

using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;


void Timer1_Callback(const ros::TimerEvent&);
void Timer1_Callback_MCU(const ros::TimerEvent&);
void Timer1_Callback_LOG_GUI(const ros::TimerEvent&);
void Callback_GUI(const ugv_msgs::toUGV::ConstPtr& msg);
void Callback_IMU(const ugv_1000::IMU_msg::ConstPtr& msg);
void Callback_GPS(const ugv_1000::GPS_msg::ConstPtr& msg);
ugv_msgs::toGUI msg1;
ros::Publisher pub1;

FrameCommad_RF FrameCommad_RFnew;
data_PID steering_PID_para;
data_PID speed_PID_para;
unsigned int mode_control, node_LOS=1, mode_PWM_PID=0,mode_IMU=1;

//Save Data
std::ofstream save_data;
float GPS_x_save[10000], GPS_y_save[10000];
float set_heading_save[10000], heading_save[10000];
int PWM_lf_save[10000], PWM_rt_save[10000], PWM_fr_save[10000], PWM_bk_save[10000];   //PWM 4 motor
int k_save = 0;
extern float ye_LOSsave;

// mappoint Mappointlatlng[6];//6  point in LOS
// xypoint Mappointxy[6];//6 point LOS in Oxy
// double Mappoint_x[6] , Mappoint_y[6];
//double Mappoint_x[6]={1.42,1.693,2.877,3.861}  , Mappoint_y[6]={0.879,2.676,3.373,5.371};


//Bspline
int num_Bspline=0	, alpha=0, order=0;
double Bspline_px[20] , Bspline_py[20] , Bspline_u[20];

// Matrix ref_path;

mappoint DataGPS_base;
double GPS_speed;
GPSSpeedFilter GPS_speed_filter;
mappoint DataGPS_latlng;
//xypoint*	DataGPS_xy; 
// double GPS_x,GPS_y;
DataGPSxy DataGPSxy1;
double Tam_x, Tam_y;
uint8_t numWaypoint = 0, numCirclepoint = 0;
TrajectoryType trajectory_type;
double Waypoint_x[10], Waypoint_y[10];
double CircleX[10], CircleY[10], Circle_w;
bool isClockwise;
DataUGV data_UGV;
uint8_t pre_mode;



/*Struct for 4 motor*/
motor_struct motor_front_struct, motor_behind_struct,motor_left_struct,motor_right_struct;

// ///////DP/////////
// mappointSi dp_origin;			//origin
// mappointSi dp_position;		//dp position
// xypointSi xy_dp_position;	//coordinate of dp position to origin
// xypointSi xy_dp_gps;			//coordinate of current gps to dp_position
// xypointSi xy_dp_gps_f;		//coordinate of current gps to dp_position after filter
// float dp_hyaw;

//PID values
double kp_ah, ki_ah, kd_ah;
double kph, kih, kdh;
double kpx, kix, kdx;
double kpy, kiy, kdy;

// /*
// convertHeading: goc heading tinh dc hien tai
// precovertHeading: goc heading tinh duoc truoc do
// KHeading: KHeading la goc heading chuan hoa sao cho goc lien tuc
// headingLOS:
// */
// volatile double convertHeading=0;
// double preIMU=0,preconvertHeading=0,KHeading=0,LOS_check=0;
// double pre_KHeading;
// double v_Heading, pre_v_Heading;		//van toc goc
// double a_Heading;//gia toc goc

volatile double headingLOS=0, KheadingLOS=0,preheadingLOS=0;
double yaw=0;

/*Framedata send to STM*/
//FrameCommad_RF FrameCommad_RF2STM;
double main_stm_temperature;

uint8_t dir_motor_front;
int duty_motor_front;
uint8_t dir_motor_behind;
int duty_motor_behind;
uint8_t dir_motor_left;
int duty_motor_left;
uint8_t dir_motor_right;
int duty_motor_right;

// Stanley paramerters
DataStanley Stanley_parameters;

// MPC parameters
time_t my_time = time(NULL);

DataMPC MPC_parameters;

double tf = 50;
double Ts = 0.1;
double k = 0;
double L = 0.6;
double u1_ref = 5/3.6;
double theta_ref = 0;
double phi_ref = 0;
double x0_input[6*1] = { 
                           -50,
                           -8,
                           30*PI_M/180,
                           0,
                           1/3.6,
                           0*PI_M/180
                         };
// double x0_input[6*1] = { 
//                            -2.0202,
//                            5,
//                            0.5236,
//                            0,
//                            -0.2778,
//                            0*PI_M/180
//                          };
Matrix x0 = Matrix(x0_input, 6U, 1U); // Zeros(6,1);


double umax_input[2*1] = { 
                           0/3.6,
                           (double)10/180*PI_M 
                         };
Matrix umax = Matrix(umax_input, 2U, 1U);
double umin_input[2*1] = { 
                           0/3.6,
                           (double)-10/180*PI_M 
                         };
Matrix umin = Matrix(umin_input, 2U, 1U);

// double ref_path_input[4*1] = { 
//                               -50,
//                               -10,
//                               0,
//                               0 
//                             };
// Matrix ref_path = Matrix(ref_path_input, 4U, 1U);

int N = 3; // number of predictive step
double Q_input[6*6] = { 
						20,     0,     0,     0,     0,     0,
						0,     20,     0,     0,     0,     0,
						0,     0,     50,     0,     0,     0,
						0,     0,     0,     0,     0,     0,
						0,     0,     0,     0,     0,     0,
						0,     0,     0,     0,     0,     0,
						};
double P_input[6*6] = { 
						300,     0,     0,     0,     0,     0,
						0,     300,     0,     0,     0,     0,
						0,     0,     300,     0,     0,     0,
						0,     0,     0,     0,     0,     0,
						0,     0,     0,     0,     0,     0,
						0,     0,     0,     0,     0,     0,
                         };
Matrix Q = Matrix(Q_input, // data
					6U, 6U); // row & col
Matrix P = Matrix(P_input, // data
					6U, 6U); // row & col

double R_input[2*2] = { 
						0.01,     0,
						0,     0.05
						};
Matrix R = Matrix(R_input, // data
					2U, 2U); // row & col

double z0_input[6*1] = { 
						-10,
						0,
						0*PI_M/180,
						0,
						3/3.6,
						0*PI_M/180,
						};
Matrix z = Matrix(z0_input, 6U, 1U);
Matrix pre_z = Matrix(z);
// Store matrix to file
//   std::filebuf fb;
//   fb.open ("test.txt",std::ios::out);
//   std::ostream os(&fb);

// Data for MCU
// data_toMCU data_PC_to_MCU;
data_toPC data_MCU_to_PC;
serial::Serial sp_MCU;



/**
 *  Main Fucntion
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Controller");
    ros::NodeHandle n;

    // if(Init_CAN())	return 1;	//Check CAN is OK?
    // ros::Rate loop_rate(1/0.015);
	ros::MultiThreadedSpinner spinner(4); // Use 4 threads

  	ros::Subscriber sub1 = n.subscribe("GPS_read", 1000, Callback_GPS);
	ros::Subscriber sub2 = n.subscribe("IMU_read", 1000, Callback_IMU);
	ros::Subscriber sub3 = n.subscribe("settings", 1000, Callback_GUI);
	pub1 = n.advertise<ugv_msgs::toGUI>("measures", 100000);
	ros::Timer timer1 = n.createTimer(ros::Duration((double)0.1), Timer1_Callback);
	ros::Timer timer2 = n.createTimer(ros::Duration((double)0.015), Timer1_Callback_MCU);
	ros::Timer timer3 = n.createTimer(ros::Duration((double)0.015), Timer1_Callback_LOG_GUI);

	// serial for MCU
	string port("/dev/ttyUSB_MCU"); //achieve name of port
	// string port("/dev/ttyUSB_IMU"); //achieve name of port
	serial::Timeout to = serial::Timeout::simpleTimeout(30);
	sp_MCU.setPort(port); // Set the name of the serial port to be opened
	sp_MCU.setBaudrate(115200); // Set the baud rate of serial communication
	sp_MCU.setTimeout(to); // Serial port set timeout
	try
	{
		sp_MCU.open();
	}
	//serial::IOException
	catch(serial::IOException const& err)
	{
		std::cout << " SerialPort for MCU: Fail" << std::endl;
	}
	if(sp_MCU.isOpen())
    	std::cout << " SerialPort for MCU: Successfully." << std::endl;
  	else
		std::cout << " SerialPort for MCU: Fail" << std::endl;	
	ros::Duration(1).sleep(); // waiting for serial is ready

	int Bspline_numpoint = 15, Bspline_order = 3, Bspline_anpha = 1;
	uint16_t i = 0; 
	uint16_t CheckSumCalValue;

	int FirstVal ;
	int SecondVal ;

	//set default values
	DataGPS_latlng.lat = 10.772992663121164;
	DataGPS_latlng.lng = 106.659779558453920;
	data_UGV.heading  = 0;


	//save data
	save_data.open("USV_data.dat");
	if(!save_data)
	{
		std::cerr<<"Error:file could not be opened"<<std::endl;
		return 1;
	}
	else
	{
		save_data<<"[UGV-MPC]"<<std::endl;
		save_data<<"[GPS_x, GPS_y, set_steering, steering, heading, steering_dot, steering_dot_delta, speed_left, u_pid_left, speed_right, u_pid_right]="<<std::endl;
	}
	std::cout<<std::endl;
	ros::Time begin = ros::Time::now();
	std::cout<<"present time:"<<begin.toSec()/60<<std::endl;

	spinner.spin();
	
	while (ros::ok())
	{		
		// loop_rate.sleep();	
		// ros::Duration(0.015).sleep();		
	}
/*
save_data<<"[Bspline_px,Bspline_py,Bspline_u]="<<std::endl;
		for (int i = 0; i < num_Bspline+order; i++)
		{
			save_data<<Bspline_px[i]<<","<<Bspline_py[i]<<","<<Bspline_u[i]<<";"<<endl;
		}
*/

    std::cout<<"............Ending............"<<std::endl;  
    return 0;

}

/**
 * Timer1_Callback() for loop control
*/
void Timer1_Callback(const ros::TimerEvent&)
{
	ROS_INFO("************While loop************");
	if(FrameCommad_RFnew.BT1 == 1)		
	{
		if (pre_mode!=FrameCommad_RFnew.BT2)
		{
			pre_mode=FrameCommad_RFnew.BT2;
			//Turn off all Motor
			// SendPWMtoMotor(100, 100, 100, 100, 0x111);
						
		}
		switch(FrameCommad_RFnew.BT2)
		{
			case 0: //manual
			{
				std::cout <<"*********** Joystick Mode *************" << std::endl;
				FrameCommad_RFnew.set_steering = duty_motor_front*30/100; // Convert -100->100 to -30->30
				FrameCommad_RFnew.set_speed = duty_motor_behind;
				std::cout <<"	Steering = " << FrameCommad_RFnew.set_steering << std::endl;
				std::cout <<"	Speed_pwm = " << FrameCommad_RFnew.set_speed << std::endl;

				// Prepare command data to MCU
				data_toMCU data_PC_to_MCU;
				data_PC_to_MCU.cmd.start = 1; // Start/Stop variable
				data_PC_to_MCU.cmd.mode = 1; // steering Position PID & Speed PWM mode
				data_PC_to_MCU.cmd.Kp_steering = steering_PID_para.Kp;
				data_PC_to_MCU.cmd.Ki_steering = steering_PID_para.Ki;
				data_PC_to_MCU.cmd.Kd_steering = steering_PID_para.Kd;
				data_PC_to_MCU.cmd.alfa = 0;
				data_PC_to_MCU.cmd.beta = 0;
				data_PC_to_MCU.cmd.L1 = 0;
				data_PC_to_MCU.cmd.L2 = 0;
				data_PC_to_MCU.cmd.Kp_speed = speed_PID_para.Kp;
				data_PC_to_MCU.cmd.Ki_speed = speed_PID_para.Ki;
				data_PC_to_MCU.cmd.Kd_speed = speed_PID_para.Kd;
				data_PC_to_MCU.cmd.speed_pwm = (float)FrameCommad_RFnew.set_speed;		

				if (FrameCommad_RFnew.set_steering > 30)
					FrameCommad_RFnew.set_steering = 30;
				else if (FrameCommad_RFnew.set_steering < -30)
					FrameCommad_RFnew.set_steering = -30;
				data_PC_to_MCU.cmd.setpoint = ((float)FrameCommad_RFnew.set_steering+30)*60/60; // scale (-20->20)->(0->58)

				uint8_t checksum = 0;
				// uint8_t len_frame = sizeof(data_PC_to_MCU.cmd);
				uint8_t len_frame = 51;
				for (int k = 0; k < len_frame-1; k++)
				{
					checksum ^= data_PC_to_MCU.data[k];
				}
				data_PC_to_MCU.data[len_frame-1] = checksum;
				// std::cout << "len_frame:" << (int)len_frame<< std::endl;
				// std::cout << "Serial_MCU:";
				// for (int k = 0; k < len_frame; k++)
				// {
				// 	std::cout << (int)data_PC_to_MCU.data[k] << ",";
				// }
				// std::cout << std::endl;
				if (sp_MCU.isOpen())
					sp_MCU.write(data_PC_to_MCU.data, len_frame); // Write command to MCU	

				break;
			}
			case 1: //PID steering mode
			{
				std::cout <<"*********** Auto heading Mode *************" << std::endl;
				std::cout <<"set_steering = " << FrameCommad_RFnew.set_steering << std::endl;
				std::cout << "steering = " << data_UGV.steering << std::endl;	
				std::cout << "set_speed = " << FrameCommad_RFnew.set_speed << std::endl;		

				// Prepare command data to MCU
				data_toMCU data_PC_to_MCU;
				data_PC_to_MCU.cmd.start = 1; // Start/Stop variable
				data_PC_to_MCU.cmd.mode = 0; // steering Position PID & Speed PWM mode
				data_PC_to_MCU.cmd.Kp_steering = steering_PID_para.Kp;
				data_PC_to_MCU.cmd.Ki_steering = steering_PID_para.Ki;
				data_PC_to_MCU.cmd.Kd_steering = steering_PID_para.Kd;
				data_PC_to_MCU.cmd.alfa = 0;
				data_PC_to_MCU.cmd.beta = 0;
				data_PC_to_MCU.cmd.L1 = 0;
				data_PC_to_MCU.cmd.L2 = 0;
				data_PC_to_MCU.cmd.Kp_speed = speed_PID_para.Kp;
				data_PC_to_MCU.cmd.Ki_speed = speed_PID_para.Ki;
				data_PC_to_MCU.cmd.Kd_speed = speed_PID_para.Kd;
				data_PC_to_MCU.cmd.speed_pwm = (float)FrameCommad_RFnew.set_speed;

				if (FrameCommad_RFnew.set_steering > 20)
					FrameCommad_RFnew.set_steering = 20;
				else if (FrameCommad_RFnew.set_steering < -20)
					FrameCommad_RFnew.set_steering = -20;
				data_PC_to_MCU.cmd.setpoint = ((float)FrameCommad_RFnew.set_steering+20)*60/40; // scale (-20->20)->(0->58)

				uint8_t checksum = 0;
				// uint8_t len_frame = sizeof(data_PC_to_MCU.cmd);
				uint8_t len_frame = 51;
				for (int k = 0; k < len_frame-1; k++)
				{
					checksum ^= data_PC_to_MCU.data[k];
				}
				data_PC_to_MCU.data[len_frame-1] = checksum;
				// std::cout << "len_frame:" << (int)len_frame<< std::endl;
				// std::cout << "Serial_MCU:";
				// for (int k = 0; k < len_frame; k++)
				// {
				// 	std::cout << (int)data_PC_to_MCU.data[k] << ",";
				// }
				// std::cout << std::endl;
				if (sp_MCU.isOpen())
					sp_MCU.write(data_PC_to_MCU.data, len_frame); // Write command to MCU				

				break;
			}
			case 2: //MPC
			{	
				std::cout << "*********** MPC Mode *************" << std::endl;
				auto millisec_since_epoch_1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
				double time = (double)k*Ts; 
				// data_UGV.speed = (int)(data_UGV.speed*100); // m/s
				// data_UGV.speed = data_UGV.speed/100;
				// data_UGV.speed = 1.2; // -> TEST

				double z_input[6*1] = { 
                          data_UGV.GPS_x, 				// x
                          data_UGV.GPS_y, 				// y
                          data_UGV.heading*PI_M/180, 	// theta (rad)
                          data_UGV.steering*PI_M/180, 	// phi (rad)
                          (data_UGV.speed_left+data_UGV.speed_right)/2, 				// u1 (m/s)
                          data_UGV.steering_dot,		// u2 (rad/s)
						//   100*PI_M/180	// u2 (rad/s)
                         };
  				z = Matrix(z_input, 6U, 1U);

				Matrix refpoint_out;
				Matrix delta_u = linear_MPC(time, Ts, z, pre_z, ref_path, 
							MPC_parameters.mpc_Q, MPC_parameters.mpc_R, MPC_parameters.mpc_P, MPC_parameters.mpc_Np,
							MPC_parameters.mpc_umax, MPC_parameters.mpc_umin, MPC_parameters.mpc_L_wheel, 
							MPC_parameters.refpoint_offset, MPC_parameters.pre_refpoint_index, refpoint_out);
				// Matrix delta_u = linear_MPC(time, Ts, z, pre_z, ref_path, 
				// 								Q, R, P, N,
				// 								umax, umin, L, z);
				if((delta_u.get_value(0,0) == 0) &&
					(delta_u.get_value(1,0) == 0) )
				{
					FrameCommad_RFnew.BT1 = 0; // stop
					return;
				}
				
				// update refpoints to GUI
				msg1.num_mpc_refpath = refpoint_out.size_cols();
				// double lat_refpoint, lng_refpoint;
				for (int k = 0; k < msg1.num_mpc_refpath; k++)
				{
					// ConverttoLatLong(refpoint_out.get_value(0,k), refpoint_out.get_value(1,k), DataGPS_base.lat, DataGPS_base.lng, lat_refpoint, lng_refpoint);
					msg1.x_mpc_repath[k] = refpoint_out.get_value(0,k);
					msg1.y_mpc_repath[k] = refpoint_out.get_value(1,k);
				}
				
				std::cout << "delta_u.print_matrix() " << std::endl;
				delta_u.print_matrix();
				data_UGV.steering_dot_delta = delta_u.get_value(1,0);
				data_UGV.steering_dot = z.get_value(5,0) + delta_u.get_value(1,0); // update next u2=steering_dot
				if(data_UGV.steering_dot > 300*PI_M/180) {
					data_UGV.steering_dot = 300*PI_M/180;
				} else if (data_UGV.steering_dot < -300*PI_M/180) {
					data_UGV.steering_dot = -300*PI_M/180;
				}
				double next_steering = z.get_value(3,0) + data_UGV.steering_dot*Ts;
				next_steering = next_steering*180/PI_M;
				std::cout << "next_steering = " << next_steering << std::endl;
				if (next_steering > 20)
					next_steering = 20;
				else if (next_steering < -20)
					next_steering = -20;
				// msg1.set_steering = next_steering;
				// data_UGV.steering = msg1.set_steering; //->TEST
				
				pre_z = z;

				// update state system to file
				// operator<<(os, z.transpose());

				// Prepare command data to MCU
				data_toMCU data_PC_to_MCU;
				data_PC_to_MCU.cmd.start = 1; // Start/Stop variable
				data_PC_to_MCU.cmd.mode = 0; // steering Position PID & Speed PID mode
				data_PC_to_MCU.cmd.Kp_steering = steering_PID_para.Kp;
				data_PC_to_MCU.cmd.Ki_steering = steering_PID_para.Ki;
				data_PC_to_MCU.cmd.Kd_steering = steering_PID_para.Kd;
				data_PC_to_MCU.cmd.setpoint = (next_steering+20)*60/40; // scale (-20->20)->(0->58)
				data_PC_to_MCU.cmd.alfa = 0;
				data_PC_to_MCU.cmd.beta = 0;
				data_PC_to_MCU.cmd.L1 = 0;
				data_PC_to_MCU.cmd.L2 = 0;
				data_PC_to_MCU.cmd.Kp_speed = speed_PID_para.Kp;
				data_PC_to_MCU.cmd.Ki_speed = speed_PID_para.Ki;
				data_PC_to_MCU.cmd.Kd_speed = speed_PID_para.Kd;
				// data_PC_to_MCU.cmd.speed_pwm = (float)FrameCommad_RFnew.set_speed;
				if(FrameCommad_RFnew.set_speed_soft_start < FrameCommad_RFnew.set_speed - 0.2) {
					FrameCommad_RFnew.set_speed_soft_start += 0.1;
					data_PC_to_MCU.cmd.speed_pwm = (float)FrameCommad_RFnew.set_speed_soft_start;
				}
				else {
					data_PC_to_MCU.cmd.speed_pwm = (float)FrameCommad_RFnew.set_speed;
				}
				uint8_t checksum = 0;
				// uint8_t len_frame = sizeof(data_PC_to_MCU.cmd);
				uint8_t len_frame = 51;
				for (int k = 0; k < len_frame-1; k++)
				{
					checksum ^= data_PC_to_MCU.data[k];
				}
				data_PC_to_MCU.data[len_frame-1] = checksum;
				if (sp_MCU.isOpen())
					sp_MCU.write(data_PC_to_MCU.data, len_frame); // Write command to MCU
				

				auto millisec_since_epoch_2 = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
				cout << "millisec_since_epoch: " << millisec_since_epoch_2-millisec_since_epoch_1 << endl;

				break;
			}
			case 3: // Stanley mode
			{
				std::cout << "*********** Stanley Mode *************" << std::endl;
				static int counter = 0;
				// data_UGV.GPS_x = 7;
				// data_UGV.GPS_y = 7;
				double z_input[6*1] = { 
                          data_UGV.GPS_x, 				// x
                          data_UGV.GPS_y, 				// y
                          data_UGV.heading*PI_M/180, 	// theta (rad)
                          data_UGV.steering*PI_M/180, 	// phi (rad)
                         };
  				z = Matrix(z_input, 4U, 1U);
				std::cout << "ref_path.get_value(3,10)  = " << ref_path.get_value(3,10) << std::endl;
				// data_UGV.speed = 1; // m/s
				double next_steering = stanley_steering(0, Ts, z, pre_z, ref_path, 
                        								(data_UGV.speed_left+data_UGV.speed_right)/2,
														// (data_UGV.speed_right), 
														Stanley_parameters.stanley_K_gain, Stanley_parameters.pre_refpoint_index);
				if(next_steering  > 250)// check is invalid value
				{
					FrameCommad_RFnew.BT1 = 0; // stop
					return;
				}
				next_steering = next_steering*180/PI_M;
				if (next_steering > 20)
					next_steering = 20;
				else if (next_steering < -20)
					next_steering = -20;
				std::cout << "next_steering = " << next_steering << std::endl;
							
				pre_z = z;

				// Prepare command data to MCU
				data_toMCU data_PC_to_MCU;
				data_PC_to_MCU.cmd.start = 1; // Start/Stop variable
				data_PC_to_MCU.cmd.mode = 0; // steering Position PID & Speed PID mode
				data_PC_to_MCU.cmd.Kp_steering = steering_PID_para.Kp;
				data_PC_to_MCU.cmd.Ki_steering = steering_PID_para.Ki;
				data_PC_to_MCU.cmd.Kd_steering = steering_PID_para.Kd;
				data_PC_to_MCU.cmd.setpoint = (next_steering+20)*60/40; // scale (-20->20)->(0->58)
				data_PC_to_MCU.cmd.alfa = 0;
				data_PC_to_MCU.cmd.beta = 0;
				data_PC_to_MCU.cmd.L1 = 0;
				data_PC_to_MCU.cmd.L2 = 0;
				data_PC_to_MCU.cmd.Kp_speed = speed_PID_para.Kp;
				data_PC_to_MCU.cmd.Ki_speed = speed_PID_para.Ki;
				data_PC_to_MCU.cmd.Kd_speed = speed_PID_para.Kd;
				// data_PC_to_MCU.cmd.speed_pwm = (float)FrameCommad_RFnew.set_speed;
				if(FrameCommad_RFnew.set_speed_soft_start < FrameCommad_RFnew.set_speed - 0.2) {
					FrameCommad_RFnew.set_speed_soft_start += 0.1;
					data_PC_to_MCU.cmd.speed_pwm = (float)FrameCommad_RFnew.set_speed_soft_start;
				}
				else {
					data_PC_to_MCU.cmd.speed_pwm = (float)FrameCommad_RFnew.set_speed;
				}
				std::cout << "speed_pwm = " << data_PC_to_MCU.cmd.speed_pwm << std::endl;
				uint8_t checksum = 0;
				// uint8_t len_frame = sizeof(data_PC_to_MCU.cmd);
				uint8_t len_frame = 51;
				for (int k = 0; k < len_frame-1; k++)
				{
					checksum ^= data_PC_to_MCU.data[k];
				}
				data_PC_to_MCU.data[len_frame-1] = checksum;
				if (sp_MCU.isOpen())
					sp_MCU.write(data_PC_to_MCU.data, len_frame); // Write command to MCU

				break;
			}
			case 4: //Motor control
			{	
				std::cout<<"motor mode"<<std::endl;
				// SendPWMtoMotor(duty_motor_left+100, duty_motor_right+100, duty_motor_front+100, duty_motor_behind+100, 0x111);
				break;
			}
			case 5: //SBG
			{
				/***********
				//int node_SBG = 2;
				static double setheading_SBG;
				double SBG_x[6], SBG_y[6], alpha_k_SBG[6];
				// for(int i =0; i<node_SBG;i++)
				// {
				// 	SBG_x[i] = Mappoint_x[i];
				// 	SBG_y[i] = Mappoint_y[i];
				// }
				SBG_x[0] = Mappoint_x[0];
				SBG_y[0] = Mappoint_y[0];
				SBG_x[1] = Mappoint_x[2];
				SBG_y[1] = Mappoint_y[2];

				//setheading_SBG = sbg(GPS_x,GPS_y,DataIMU1.data_Yaw,Mappoint_x[1], Mappoint_y[1],SBG_x,SBG_y,node_SBG);
				sbg(SBG_x,SBG_y,alpha_k_SBG,Mappoint_x[1],Mappoint_y[1]);
				setheading_SBG = LOSGuidance_curve(SBG_x, SBG_y, alpha_k_SBG, GPS_x,GPS_y, 6);
				PID_Heading_DC_mode23_1(setheading_SBG, DataIMU1.data_Yaw,kph,kdh,kih);
				std::cout<<"GPS_x="<<GPS_x<<std::endl;
				std::cout<<"GPS_y="<<GPS_y<<std::endl;
				std::cout<<"SBG_x[0]="<<SBG_x[0]<<std::endl;
				std::cout<<"SBG_x[0]="<<SBG_y[0]<<std::endl;
				std::cout<<"SBG_x[1]="<<SBG_x[1]<<std::endl;
				std::cout<<"SBG_y[1]="<<SBG_y[1]<<std::endl;
				std::cout<<"vatcan_x="<<Mappoint_x[1]<<std::endl;
				std::cout<<"vatcan_y="<<Mappoint_y[1]<<std::endl;

				std::cout<<"setheading_SBG="<<setheading_SBG<<std::endl;

				SendPWMtoMotor(FrameCommad_RFnew.set_speed+100, FrameCommad_RFnew.set_speed+100, motor_front_struct.duty+100, 0+100, 0x111);

				msg1.setyaw = setheading_SBG;
									************/
				break;
			}
			case 6: // Bspline
			{
				//variables for Bspline	
				int Bspline_numpoint = 1000;				
				double Bspline_xd[Bspline_numpoint], Bspline_yd[Bspline_numpoint], Bspline_psid[Bspline_numpoint];																																										
				static double set_heading_LOS = 0;
				static int k_speed=0;

				// set_heading_LOS = LOSGuidance(Mappointxy, DataGPS_xy, node_LOS);

				// PID_Heading_DC_mode23(FrameCommad_RFnew.setyaw, DataIMU1.data_Yaw,kph,kdh,kih);
				std::cout<<"+++++++LOS mode++++++++++++++++++++++++++++++++++"<<std::endl;

				std::cout<<"num_Bspline="<<num_Bspline<<std::endl;
				std::cout<<"GPS_x="<<DataGPSxy1.GPS_x<<std::endl;
				std::cout<<"GPS_y="<<DataGPSxy1.GPS_y<<std::endl;

				// std::cout<<"Tam_x="<<Tam_x<<std::endl;
				std::cout<<"___LOS____"<<std::endl;
				set_heading_LOS = LOSGuidance_continous(Bspline_u, Bspline_px, Bspline_py, DataGPSxy1.GPS_x, DataGPSxy1.GPS_y,  num_Bspline, order);	
				std::cout<<"setheading="<<set_heading_LOS<<std::endl;
				//msg1.setyaw = set_heading_LOS;
				msg1.set_steering = motor_front_struct.duty;
				// PID_Heading_DC_mode23_1(set_heading_LOS, DataIMU1.data_Yaw,kph,kdh,kih);

				// SendPWMtoMotor(FrameCommad_RFnew.set_speed*k_speed/50+100, FrameCommad_RFnew.set_speed*k_speed/50+100, motor_front_struct.duty+100, 0+100,0x111);
				k_speed++;
				if(k_speed>=50)  k_speed = 50;		
				break;
			}
			case 7: //Xac dinh mo hinh USV
			{
				break;
			}
			case 8: //LOS+DP diem cuoi
			{
				break;
			}
			default:
				break;		
		}
	}
	else
	{
		//Turn off all Motor
		// Prepare command data to MCU
		data_toMCU data_PC_to_MCU;
		data_PC_to_MCU.cmd.start = 0; // Start/Stop variable
		data_PC_to_MCU.cmd.mode = 0; // steering Position PID mode
		data_PC_to_MCU.cmd.Kp_steering = 0;
		data_PC_to_MCU.cmd.Ki_steering = 0;
		data_PC_to_MCU.cmd.Kd_steering = 0;
		data_PC_to_MCU.cmd.setpoint = 0;
		data_PC_to_MCU.cmd.alfa = 0;
		data_PC_to_MCU.cmd.beta = 0;
		data_PC_to_MCU.cmd.L1 = 0;
		data_PC_to_MCU.cmd.L2 = 0;
		data_PC_to_MCU.cmd.Kp_speed = 0;
		data_PC_to_MCU.cmd.Ki_speed = 0;
		data_PC_to_MCU.cmd.Kd_speed = 0;
		data_PC_to_MCU.cmd.speed_pwm = 0;
		FrameCommad_RFnew.set_speed_soft_start = 0;
		
		uint8_t checksum = 0;
		// uint8_t len_frame = sizeof(data_PC_to_MCU.cmd);
		uint8_t len_frame = 51;
		for (int k = 0; k < len_frame-1; k++)
		{
			checksum ^= (uint8_t)data_PC_to_MCU.data[k];
		}
		data_PC_to_MCU.data[len_frame-1] = checksum;
		// std::cout << "len_frame:" << (int)len_frame<< std::endl;
		// std::cout << "Serial_MCU:";
		// for (int k = 0; k < len_frame; k++)
		// {
		// 	std::cout << (int)data_PC_to_MCU.data[k] << ",";
		// }
		// std::cout << std::endl;
		// 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,
		if (sp_MCU.isOpen())
			sp_MCU.write(data_PC_to_MCU.data, len_frame); // Write command to MCU

		// msg1.lat_refpoint = 0;
		// msg1.lng_refpoint = 0;
		msg1.num_mpc_refpath = 0;
		// double lat_refpoint, lng_refpoint;
		for (int k = 0; k < 20; k++)
		{
			// ConverttoLatLong(refpoint_out.get_value(0,k), refpoint_out.get_value(1,k), DataGPS_base.lat, DataGPS_base.lng, lat_refpoint, lng_refpoint);
			msg1.x_mpc_repath[k] = 0;
			msg1.y_mpc_repath[k] = 0;
		}

		

	}

}


/**
 * Timer1_Callback_MCU() for MCU
*/
// Data structure: 
// "<_setsteering> <_steering> <_u> <_set_rear_speed> <_speed_left> <_u_pid_left> <_speed_right>  <_u_pid_right>*<checksum>\n"
//       0              1        2           3              4             5             6                 7
void Timer1_Callback_MCU(const ros::TimerEvent&)
{
	static int k_steering_dot = 0;
	if(sp_MCU.isOpen())
	{
		// ROS_INFO("Timer1_Callback_MCU()");
		std::vector<string> subdata;
		try
		{
			std::string data = sp_MCU.readline();
			// cout<<"sp_MCU.readline():" << data <<endl;
			if(data.size() < 5) {
				// ROS_ERROR("Serial IMU: data.size() < 5");
				return;
			}
			boost::split(subdata, data, boost::is_any_of(" ")); 	
		}
		catch(serial::IOException const& err)
		{
			cout<<"Cannot navigate..."<<endl;
		}

		// update msg to GUI
		msg1.set_steering = std::strtod(subdata[0].c_str(),0);; // (deg) 
		msg1.set_steering = ((double)msg1.set_steering-30)*20/30; // scale (0->58)->(-20->20)

		msg1.steering = std::strtod(subdata[1].c_str(),0);; // (deg)
		msg1.steering = ((double)msg1.steering-30)*20/30; // scale (0->58)->(-20->20)
		data_UGV.steering = msg1.steering;
		// k_steering_dot++;
		// if(k_steering_dot > 6)
		// {
		// 	data_UGV.steering_dot = (msg1.steering - data_UGV.pre_steering)/0.015/6; //(deg/s)
		// 	data_UGV.pre_steering = msg1.steering;
		// 	k_steering_dot = 0;
		// }
		// msg1.steering = data_UGV.steering_dot;

#if defined USE_SPEED_ENCODER
		msg1.set_speed = std::strtod(subdata[3].c_str(),0)/100; // (m/s) 
		msg1.speed_left = std::strtod(subdata[4].c_str(),0)/100; // (m/s)
		data_UGV.speed_left = msg1.speed_left; 
		data_UGV.u_pid_left = std::strtod(subdata[5].c_str(),0); // (%PWM)
		msg1.speed_right = std::strtod(subdata[6].c_str(),0)/100; // (m/s) 
		data_UGV.speed_right = msg1.speed_right;
		data_UGV.u_pid_right = std::strtod(subdata[7].c_str(),0); // (%PWM)

		// msg1.set_steering += 0.01;
		// msg1.steering += 0.01;
		#endif
	}
}


/**
 * Timer1_Callback_LOG_GUI() for save log & pub msg to GUI
*/
void Timer1_Callback_LOG_GUI(const ros::TimerEvent&)
{
	// ROS_INFO("Timer1_Callback_LOG_GUI()");

	float Bat = 12;
	float I = 0.23;
	//Send data to GUI
	msg1.lat_postion = data_UGV.lat;
	msg1.lng_postion = data_UGV.lng;
	msg1.yaw = data_UGV.heading; //deg
	// msg1.yaw += 1;
	//msg1.setyaw = FrameCommad_RFnew.setyaw;
	//Bat -= 0.001;
	msg1.V_battery = Bat;
	//I += 0.01;
	msg1.I_system = I;
	msg1.leak_sensor = 0;

	pub1.publish(msg1);
	// ros::spinOnce();
	/*Delay 7ms*/
	// ros::Duration(0.07).sleep();	

	//save data
// [GPS_x, GPS_y, set_steering, steering, heading, steering_dot, steering_dot_delta, speed_left, u_pid_left, speed_right, u_pid_right]
	if(FrameCommad_RFnew.BT1 == 1)
	{
		save_data<<"	"<<data_UGV.GPS_x<<","<<data_UGV.GPS_y<<","<<msg1.set_steering<<","<<data_UGV.steering<<","<<data_UGV.heading<<","<<data_UGV.steering_dot<<","<<data_UGV.steering_dot_delta<<","<<data_UGV.speed_left<<","<<data_UGV.u_pid_left<<","<<data_UGV.speed_right<<","<<data_UGV.u_pid_right<<endl;
	}
}


void Callback_GUI(const ugv_msgs::toUGV::ConstPtr& msg)
{
	if(msg->status == 1)
	{
		FrameCommad_RFnew.status = msg->status;
		FrameCommad_RFnew.set_steering = msg->set_steering;
		FrameCommad_RFnew.BT1 = msg->BT1;
		FrameCommad_RFnew.BT2 = mode_control = msg->BT2;
		FrameCommad_RFnew.BT3 = num_Bspline = msg->BT3;
		FrameCommad_RFnew.BT4 = mode_IMU = msg->BT4;
		FrameCommad_RFnew.BT5 = mode_PWM_PID = msg->BT5;
		FrameCommad_RFnew.joyx = msg->joyx;
		FrameCommad_RFnew.joyy = msg->joyy;
		FrameCommad_RFnew.set_speed = msg->set_speed;
		

		duty_motor_front = msg->duty_motor_front;

		duty_motor_behind = msg->duty_motor_behind;

		duty_motor_left = msg->duty_motor_left;

		duty_motor_right = msg->duty_motor_right;

		//PID values 
		// kp_ah = msg->kp_ah;
		// ki_ah = msg->ki_ah; 
		// kd_ah = msg->kd_ah;

		steering_PID_para.Kp = msg->Kp_steering; 
		steering_PID_para.Ki = msg->Ki_steering; 
		steering_PID_para.Kd = msg->Kd_steering;

		speed_PID_para.Kp = msg->Kp_speed; 
		speed_PID_para.Ki = msg->Ki_speed; 
		speed_PID_para.Kd = msg->Kd_speed;

		// std::cout<<kph<<std::endl;
		// std::cout<<kdh<<std::endl;
		// std::cout<<kih<<std::endl;

		// kpx = msg->kpx; 
		// kix = msg->kix; 
		// kdx = msg->kdx;

		// kpy = msg->kpy; 
		// kiy = msg->kiy; 
		// kdy = msg->kdy;

		// reset parameters
		MPC_parameters.pre_refpoint_index = 0;
		Stanley_parameters.pre_refpoint_index = 0;

	}
	else if(msg->status == 2)	//update PID values
	{
		//PID values 
		// kp_ah = msg->kp_ah;
		// ki_ah = msg->ki_ah; 
		// kd_ah = msg->kd_ah;

		steering_PID_para.Kp = msg->Kp_steering; 
		steering_PID_para.Ki = msg->Ki_steering; 
		steering_PID_para.Kd = msg->Kd_steering;
		speed_PID_para.Kp = msg->Kp_speed; 
		speed_PID_para.Ki = msg->Ki_speed; 
		speed_PID_para.Kd = msg->Kd_speed;
		std::cout<<"*************Update PID values**********"<<std::endl;
		std::cout <<"kp_steering = " << steering_PID_para.Kp <<std::endl;
		std::cout<< "kd_steering = " << steering_PID_para.Ki <<std::endl;
		std::cout<< "ki_steering = " << steering_PID_para.Kd <<std::endl;
		std::cout <<"kp_speed= " << speed_PID_para.Kp <<std::endl;
		std::cout<< "kd_speed = " << speed_PID_para.Ki <<std::endl;
		std::cout<< "ki_speed = " << speed_PID_para.Kd <<std::endl;

		
		
		FrameCommad_RFnew.set_speed = msg->set_speed;
		std::cout<< "set_speed = " << FrameCommad_RFnew.set_speed <<std::endl;

		// kpx = msg->kpx; 
		// kix = msg->kix; 
		// kdx = msg->kdx;

		// kpy = msg->kpy; 
		// kiy = msg->kiy; 
		// kdy = msg->kdy;
	}
	else if(msg->status == 0xFF) // clear datalof
	{
		// clear file data
		save_data.close();
		save_data.open("USV_data.dat");
		save_data<<"[UGV-MPC]"<<std::endl;
		save_data<<"[GPS_x, GPS_y, set_steering, steering, heading, steering_dot, steering_dot_delta, speed_left, u_pid_left, speed_right, u_pid_right]="<<std::endl;
	}
	else
	{

	}



	if(mode_control == 2 || mode_control == 3)
	{
		// alpha = msg->alpha;
		// order = msg->order;

		DataGPS_base.lat = msg->lat_base;
		DataGPS_base.lng = msg->lng_base;
		// update waypoints
		trajectory_type = (TrajectoryType)msg->TrajectoryType;
		numWaypoint = msg->numWaypoint;
		for (int i = 0; i < 10; i++)
		{
			Waypoint_x[i] = msg->X_point[i];
			Waypoint_y[i] = msg->Y_point[i];
			CircleX[i] = msg->CircleX[i];
			CircleY[i] = msg->CircleY[i];
		}
		numCirclepoint = msg->numCirclepoint;
		Circle_w = msg->Circle_w;
		isClockwise = (bool) msg->isClockwise;

		TrajectoryPath trajectoty_path;
		trajectoty_path.setInputPointsXY(Waypoint_x, Waypoint_y, numWaypoint);
		// trajectoty_path.setCircleParametersXY(CircleX[0], CircleY[0], Circle_w, (bool)isClockwise); // O1 is Circle point
		trajectoty_path.setCircleParametersXY(CircleX, CircleY, numCirclepoint, Circle_w, (bool)isClockwise); // O1 is Circle point
		// std::cout<<"setParameters(): "<<std::endl;
		trajectoty_path.setParameters((TrajectoryType)trajectory_type);
		ref_path = trajectoty_path.generateTrajectoryPath();
		// std::cout<<"********ref_path.print_matrix()**********"<<std::endl;
		// ref_path.print_matrix();

		// Update waypoints & ref path to data log
		if(FrameCommad_RFnew.BT1 == 1)
		{	
			// update waypoint to data log
			save_data<<"% /*************************************************************************************************************/"<<std::endl;
			save_data<<"% /*************************************************************************************************************/"<<std::endl;
			save_data<<"% /*************************************************************************************************************/"<<std::endl;
			save_data<<"% [Waypoint_x, Waypoint_y]="<<std::endl;
			save_data<<"Waypoints = "<<std::endl;
			save_data<<"[						"<<std::endl;
			for (int i = 0; i < 10; i++)
			{
				save_data<<"	"<<Waypoint_x[i]<<", "<<Waypoint_y[i]<<"; "<<std::endl;
			}
			save_data<<"				];"<<std::endl;

			// update ref path to data log
			save_data<<"% [CircleX, CircleY, Circle_w, isClockwise]="<<std::endl;
			for (int i = 0; i < 10; i++)
			{
				save_data<<"	"<<CircleX[i]<<", "<<CircleY[i]<<"; "<<std::endl;
			}
			// save_data<<"Circle_para="<<"["<<CircleX[0]<<", "<<CircleY<<", "<<Circle_w<<", "<<isClockwise<<"];"<<std::endl;
			save_data<<"% [ref_path: 4xN]="<<std::endl;
			save_data<<"ref_path = "<<std::endl;
			save_data<<"[						"<<std::endl;
			for (int i = 0; i < ref_path.size_rows(); ++i) {
				save_data << ref_path.get_value(i,0);
				for (int j = 1; j < ref_path.size_cols(); ++j) {
					save_data << ", " << ref_path.get_value(i,j);
				}
				save_data << "; " << endl;
			}
			save_data<<"								];"<<std::endl;
			save_data<<"% [GPS_x, GPS_y, set_steering, steering, heading, speed, steering_dot]="<<std::endl;
			save_data<<"UGV_data = "<<std::endl;
			save_data<<"[						"<<std::endl;
		}
		else
		{
			save_data<<"									];"<<std::endl;
		}
	}
	if(mode_control == 2)	//MPC mode
	{
		// free old data
		// MPC_parameters.mpc_Q.deallocSpace();
		// MPC_parameters.mpc_P.deallocSpace();
		// MPC_parameters.mpc_R.deallocSpace();
		// MPC_parameters.mpc_umax.deallocSpace();
		// MPC_parameters.mpc_umin.deallocSpace();

		std::cout<<"*************Update MPC Parameters**********"<<std::endl;

		MPC_parameters.mpc_L_wheel = msg->mpc_L_wheel;
		MPC_parameters.mpc_Np = msg->mpc_Np;
		MPC_parameters.mpc_Nu = msg->mpc_Nu;

		double Q_input[6*6] = { 
								0,     0,     0,     0,     0,     0,
								0,     0,     0,     0,     0,     0,
								0,     0,     0,     0,     0,     0,
								0,     0,     0,     0,     0,     0,
								0,     0,     0,     0,     0,     0,
								0,     0,     0,     0,     0,     0,
								};
		double P_input[6*6] = { 
								30,     0,     0,     0,     0,     0,
								0,     30,     0,     0,     0,     0,
								0,     0,     30,     0,     0,     0,
								0,     0,     0,     0,     0,     0,
								0,     0,     0,     0,     0,     0,
								0,     0,     0,     0,     0,     0,
								};
		for (int i = 0; i < 4; i++)
		{
			Q_input[i+6*i] = msg->mpc_Q[i]; // update weight of x, y, theta, phi in Q matrix
		}
		MPC_parameters.mpc_Q = Matrix(Q_input, // data
										6U, 6U); // row & col
		MPC_parameters.mpc_P = Matrix(Q_input, // data
                      					6U, 6U); // row & col

		double R_input[2*2] = { 
                          0.01,     0,
                          0,     0.05
                         };
		for (int i = 0; i < 2; i++)
		{
			R_input[i+2*i] = msg->mpc_R[i]; // update weight of delta_u1, delta_u2 in R matrix
		}
		MPC_parameters.mpc_R = Matrix(R_input, // data
                      					2U, 2U); // row & col

		double umax_input[2*1] = { 
                           0,
                           (float)10/180*PI_M 
                         };
		double umin_input[2*1] = { 
								0,
								(float)-10/180*PI_M 
								};
		for (int i = 0; i < 2; i++)
		{
			umax_input[i] = msg->mpc_umax[i]; // update max of delta_u1, delta_u2
			umin_input[i] = msg->mpc_umin[i]; // update max of delta_u1, delta_u2
		}
		MPC_parameters.mpc_umax = Matrix(umax_input, 2U, 1U);
		MPC_parameters.mpc_umin = Matrix(umin_input, 2U, 1U);
		MPC_parameters.refpoint_offset = msg->refpoint_offset;

	}
	else if(mode_control == 3)	// Stanley mode
	{
		std::cout<<"*************Update Stanley Parameters**********"<<std::endl;
		// Stanley parameters
		Stanley_parameters.stanley_L_wheel = msg->stanley_L_wheel;
		Stanley_parameters.stanley_K_gain = msg->stanley_K_gain;
		std::cout<< "Stanley_parameters.stanley_L_wheel = " << Stanley_parameters.stanley_L_wheel <<std::endl;
		std::cout<< "Stanley_parameters.stanley_K_gain = " << Stanley_parameters.stanley_K_gain <<std::endl;
		 
	}
}

void Callback_IMU(const ugv_1000::IMU_msg::ConstPtr& msg)
{
	// DataIMU1.data_Roll = Convertheading(msg->roll, mode_IMU);
	// DataIMU1.data_Pitch = Convertheading(msg->pitch, mode_IMU);
	// DataIMU1.data_Yaw = Convertheading(msg->yaw, mode_IMU);
	data_UGV.heading = msg->yaw;
	//std::cout<<"yaw"<<DataIMU1.data_Yaw;
  	// ROS_INFO("I heard: [%f]", data_UGV.heading);
}

double movingAvg(int window_size, double nextVal)
{
  static double Sum = 0;
  static int pos = 0;
  static double gps_speed_arr[100] = {0};
  //Subtract the oldest number from the prev sum, add the new number
  Sum = Sum - gps_speed_arr[window_size-1] + nextVal; //remove last index & add new value in 'Sum'
  
  for(int k=window_size-1; k>0; k--){
	gps_speed_arr[k] = gps_speed_arr[k-1];
  }
  gps_speed_arr[0] = nextVal; //Assign the nextVal to the first position in the array
  //return the average
  return (double) Sum / window_size;
}

void Callback_GPS(const ugv_1000::GPS_msg::ConstPtr& msg)
{
	DataGPS_latlng.lat = msg->lat;
	DataGPS_latlng.lng = msg->lng;
	// GPS_speed = msg->speed;
	data_UGV.lat = DataGPS_latlng.lat;
	data_UGV.lng = DataGPS_latlng.lng;
	//DataGPS_xy.x = msg->x;
	//DataGPS_xy.y = msg->y;

	// data_UGV.speed_left = movingAvg(5, msg->speed*0.514444); // convert knot/s to m/s
	// data_UGV.speed_right = data_UGV.speed_left;
#if defined USE_SPEED_GPS
	// GPS_speed_filter.speed[GPS_speed_filter.index++] = msg->speed*0.514444; // convert knot/s to m/s
	// if(GPS_speed_filter.index > SPEED_GPS_INDEX_FILTER_MAX)
	// {
	// 	GPS_speed = 0;
	// 	for (int k = 0; k < SPEED_GPS_INDEX_FILTER_MAX; k++)
	// 	{
	// 		GPS_speed += GPS_speed_filter.speed[k];
	// 	}
		
	// 	data_UGV.speed = GPS_speed/SPEED_GPS_INDEX_FILTER_MAX;
	// 	msg1.speed = data_UGV.speed;
	// 	GPS_speed_filter.index = 0; //reset index
	// }
	data_UGV.speed_left = movingAvg(5, msg->speed*0.514444); // convert knot/s to m/s
	data_UGV.speed_right = data_UGV.speed_left;
	msg1.speed_left = data_UGV.speed;
	msg1.speed_right = data_UGV.speed;
#endif

	if((DataGPS_base.lat != 0) && (DataGPS_base.lng != 0))// check data lat,lng is valid?
	{
		// convertLatLong2XY_GPS(DataGPS_base, DataGPS_latlng, &DataGPSxy1.GPS_x, &DataGPSxy1.GPS_y, true );
		// Convert2XY(DataGPS_latlng.lat,DataGPS_latlng.lng,DataGPS_base.lat,DataGPS_base.lng,&Tam_x, &Tam_y);
		ConverttoXY(DataGPS_latlng.lat, DataGPS_latlng.lng, DataGPS_base.lat, DataGPS_base.lng, DataGPSxy1.GPS_x, DataGPSxy1.GPS_y);
		if(FrameCommad_RFnew.BT2 == 2) // MPC mode
		{
			// // Vu TEST
			// // cur_x = z(0) - L*cos(z(2));                   % x position of rear axle       [m]
			// // cur_y = z(1) - L*sin(z(2));                   % y position of rear axle       [m]
			// DataGPSxy1.GPS_x = DataGPSxy1.GPS_x - MPC_parameters.mpc_L_wheel*cos(data_UGV.heading*PI_M/180);
			// DataGPSxy1.GPS_y = DataGPSxy1.GPS_y - MPC_parameters.mpc_L_wheel*sin(data_UGV.heading*PI_M/180);
			// // update new position to GUI
			// double lat_new, lng_new;
			// ConverttoLatLong(DataGPSxy1.GPS_x, DataGPSxy1.GPS_y, DataGPS_base.lat, DataGPS_base.lng, lat_new, lng_new);
			// data_UGV.lat = lat_new;
			// data_UGV.lng = lng_new;
		}

	}
	data_UGV.GPS_x = DataGPSxy1.GPS_x;
	data_UGV.GPS_y = DataGPSxy1.GPS_y;
	//ROS_INFO("I heard: [%f],[%f]", DataGPS_latlng.lat,DataGPS_latlng.lng);
}