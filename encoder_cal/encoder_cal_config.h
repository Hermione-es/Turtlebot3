#ifndef ENCODER_CAL_CONFIG_H_
#define ENCODER_CAL_CONFIG_H_

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <Dynamixel2Arduino.h>
#include <MPU9250.h>
#include "encoder_cal.h"

#define THICK_F_10000                    ((WHEEL_RAD*2*PI)/(GEAR_RATIO*RESOLUTION)*10000)


//DEBUG     
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial

//Callback function prototypes
void twistMessageReceived(const geometry_msgs::Twist& msg);

//Fuction
void publishMsg();
void coordinate();
void _init_();
void backOrigin();

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> teleop_sub("cmd_vel",&twistMessageReceived);

/*******************************************************************************
* Publisher
*******************************************************************************/
// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("kalamn_orientation", &imu_msg);

// Pose of Turtlebot3
geometry_msgs::PoseStamped pose_msg;
ros::Publisher pose_pub("pose", &pose_msg);

//Yaw of Turtlebot3
std_msgs::Float64 yaw_msg;
ros::Publisher yaw_pub("yaw", &yaw_msg);

//tf of Turtlebot3
geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;


/*******************************************************************************
* Timer of Turtlebot3
*******************************************************************************/
const uint8_t PERIOD = 20;
double dt = 0.02;
static uint32_t pre_time;

/*******************************************************************************
* Back to origin
*******************************************************************************/
int finish = 0;
double lin_x = MAX_LIN_X / 2;
double ang_z = MAX_ANG_Z / 2;
double theta;
double time2turn;
double time2go;
double time2end1;
double time2end2;

/*******************************************************************************
* Calculation for coordinate
*******************************************************************************/
double encoder_yaw = 0;
double encoder_yaw_q[4];
double encoder_yaw_new = 0;

double prev_Lpos=0;
double prev_Rpos=0;
double cnt_Lpos;
double cnt_Rpos;
double dif_Lpos;
double dif_Rpos;
double dis_L;
double dis_R;
double dis = 0;
double dxl_x = 0;
double dxl_y = 0;

/*******************************************************************************
* Declaration for velocity
*******************************************************************************/
double L_Vel;
double R_Vel;
double L_RPM;
double R_RPM;
double lin_Vel;
double ang_Vel;

double set_lin;
double set_ang;
double set_Lvel;
double set_Rvel;
double set_LRPM;
double set_RRPM;

#endif // ENCODER_CAL_CONFIG_H_
