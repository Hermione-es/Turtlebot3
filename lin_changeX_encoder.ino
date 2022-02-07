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

cMPU9250 mpu;

#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial

///////////////////////////////////////
const uint8_t PERIOD = 20;
double dt = 0.02;
///////////////////////////////////////
const uint8_t DXL_DIR_PIN = 84;
const uint8_t RIGHT_ID = 1;
const uint8_t LEFT_ID = 2;
const double DXL_PROTOCOL_VERSION = 2.0;

double WHEEL_WID = 0.16; //turtlebot3_burger's width
double WHEEL_RAD = 0.033; //turtlebot3_burger's radius
double G = 1; // turtlebot3 gear ratio (XL430-W250)
double Re = 4096; // turtlebot3 encoder's resolution (XL430-W250)
double thick_F = (WHEEL_RAD*2*PI)/(G*Re) *10000; //1thick's distance

double L_Vel;
double R_Vel;
double L_RPM;
double R_RPM;
double lin_Vel;
double ang_Vel;

int finish = 0;
double lin_x = 0.22 / 2;//MAX_LIN_X(meter/sec)
double ang_z = 2.82 / 2;//MAX_ANG_Z(radian/sec)
double theta;
double time2turn;
double time2go;
double time2end1;
double time2end2;

double encoder_yaw = 0;
double encoder_yaw_q[4];
double encoder_yaw_new = 0;
double cnt_Lpos;
double cnt_Rpos;
double prev_Lpos=0;
double prev_Rpos=0;
double dif_Lpos;
double dif_Rpos;
double dis_L;
double dis_R;
double dis = 0;
double dxl_x_new;
double dxl_y_new;

double yaw = 0;
double dxl_x = 0;
double dxl_y = 0;
double dxl_theta = 0;
double set_lin;
double set_ang;
double set_Lvel;
double set_Rvel;
double set_LRPM;
double set_RRPM;


Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

void twistMessageReceived(const geometry_msgs::Twist& msg);

ros::NodeHandle nh; //ROS NodeHandle
ros::Subscriber<geometry_msgs::Twist> teleop_sub("cmd_vel",&twistMessageReceived); //Subscriber

sensor_msgs::Imu imu_msg;// IMU of Turtlebot3
ros::Publisher imu_pub("kalamn_orientation", &imu_msg);

geometry_msgs::PoseStamped pose_msg;// Pose of Turtlebot3
ros::Publisher pose_pub("pose", &pose_msg);

std_msgs::Float64 yaw_msg;//Yaw of Turtlebot3
ros::Publisher yaw_pub("yaw", &yaw_msg);

geometry_msgs::TransformStamped tfs_msg; //tf of Turtlebot3
tf::TransformBroadcaster tfbroadcaster;


void setup()
{
  Serial.begin(115200);

  nh.initNode();
  
  nh.subscribe(teleop_sub);
  
  nh.advertise(imu_pub);
  nh.advertise(pose_pub);
  nh.advertise(yaw_pub);
  
  tfbroadcaster.init(nh);

  //Dynamixel setting
  mpu.begin();

  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(RIGHT_ID);
  dxl.ping(LEFT_ID);
  dxl.torqueOff(LEFT_ID);
  dxl.torqueOff(RIGHT_ID);
  dxl.setOperatingMode(LEFT_ID, OP_VELOCITY);
  dxl.setOperatingMode(RIGHT_ID, OP_VELOCITY);
  dxl.torqueOn(LEFT_ID);
  dxl.torqueOn(RIGHT_ID);
  dxl.writeControlTableItem(DRIVE_MODE, LEFT_ID, 0);
  dxl.writeControlTableItem(DRIVE_MODE, RIGHT_ID, 0);


  //initial value set
  prev_Lpos = dxl.getPresentPosition(LEFT_ID);
  prev_Rpos = dxl.getPresentPosition(RIGHT_ID);

  encoder_yaw = 0;
  encoder_yaw_q[0] = 0;
  encoder_yaw_q[1] = 0;
  encoder_yaw_q[2] = 0;
  encoder_yaw_q[3] = 0;
  dxl_x = 0;
  dxl_y = 0;

  if (set_ang == 0) lin_drive();
  if (set_lin == 0) ang_drive();
  
}

void loop()
{

  static uint32_t pre_time;
  if (millis()-pre_time >= PERIOD)
  {
    pre_time = millis();
    coordinate();
  }

  if (millis() <= 300000)
  {
    if (set_ang == 0) lin_drive();
    if (set_lin == 0) ang_drive();
  }
  else if (millis() > 30000) backOrigin();
  
  nh.spinOnce();
}

void twistMessageReceived(const geometry_msgs::Twist& msg)
{
  
  set_lin = msg.linear.x;
  set_ang = msg.angular.z;
  set_Lvel = set_lin + (set_ang * WHEEL_WID / 2); //target_speed_l
  set_Rvel = set_lin - (set_ang * WHEEL_WID / 2); //target_speed_r
  set_LRPM = set_Lvel * 60 / 2 / PI / WHEEL_RAD; //RAW to RPM
  set_RRPM = set_Rvel * 60 / 2 / PI / WHEEL_RAD; //RAW to RPM
}

void lin_drive()
{
  
  dxl.setGoalVelocity(LEFT_ID, set_LRPM, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, set_RRPM, UNIT_RPM);
  delay(5000);

  dxl.setGoalVelocity(LEFT_ID, -set_LRPM, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, -set_RRPM, UNIT_RPM);
  delay(10000);

  dxl.setGoalVelocity(LEFT_ID, set_LRPM, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, set_RRPM, UNIT_RPM);
  delay(5000);

  while ((encoder_yaw*180/PI) <= 90)
  {
    dxl.setGoalVelocity(LEFT_ID, 10);
    dxl.setGoalVelocity(RIGHT_ID, -10);
  }

  dxl.setGoalVelocity(LEFT_ID, set_LRPM, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, set_RRPM, UNIT_RPM);
  delay(5000);

  dxl.setGoalVelocity(LEFT_ID, -set_LRPM, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, -set_RRPM, UNIT_RPM);
  delay(10000);

  dxl.setGoalVelocity(LEFT_ID, set_LRPM, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, set_RRPM, UNIT_RPM);
  delay(5000);

  while ((encoder_yaw*180/PI) >= 0)
  {
    dxl.setGoalVelocity(LEFT_ID, -10);
    dxl.setGoalVelocity(RIGHT_ID, 10);
  }
}

void ang_drive()
{
  dxl.setGoalVelocity(LEFT_ID, 10);
  dxl.setGoalVelocity(RIGHT_ID, 10);
  delay(2000);

  dxl.setGoalVelocity(LEFT_ID, set_LRPM, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, set_RRPM, UNIT_RPM);
}

void backOrigin()
{
  dxl.setGoalVelocity(LEFT_ID, 0); //turtlebot3 Pause
  dxl.setGoalVelocity(RIGHT_ID, 0);

  dis = sqrt(pow(abs(dxl_x),2) + (pow(abs(dxl_y),2)));
  
  if  (dxl_x >= 0 and dxl_y >= 0) //case 1: +0
  {
    theta = atan2(abs(dxl_y),abs(dxl_x));
  }      
  else if(dxl_x >= 0 && dxl_y <  0) //case 2: -0
  {
    theta = -atan2(abs(dxl_y),abs(dxl_x));
  }
  else if(dxl_x <  0 && dxl_y <  0) //case 3: -(pi-0)
  {
    theta = -(PI - atan2(abs(dxl_y),abs(dxl_x)));
  }
  else if(dxl_x <  0 and dxl_y >= 0) //case 4: (pi-0)
  {
    theta = PI - atan2(abs(dxl_y),abs(dxl_x));
  }

  time2turn = theta / ang_z; //time to turn
  time2go = dis / lin_x; //time to go
  time2end1 = millis() + time2turn; //time to finish going
  
  while(millis() <= time2end1)
  {
    set_Lvel = (ang_z * WHEEL_WID / 2); //target_speed_l
    set_Rvel = - (ang_z * WHEEL_WID / 2); //target_speed_r
    set_LRPM = set_Lvel * 60 / 2 / PI / WHEEL_RAD; //RAW to RPM
    set_RRPM = set_Rvel * 60 / 2 / PI / WHEEL_RAD; //RAW to RPM
    
    dxl.setGoalVelocity(LEFT_ID, set_LRPM, UNIT_RPM);
    dxl.setGoalVelocity(RIGHT_ID, set_RRPM, UNIT_RPM);
  }

  time2end2 = millis() + time2go; //time to finish turning
  while(millis() <= time2end2)
  {
    set_Lvel = set_lin; //target_speed_l
    set_Rvel = set_lin; //target_speed_r
    set_LRPM = set_Lvel * 60 / 2 / PI / WHEEL_RAD; //RAW to RPM
    set_RRPM = set_Rvel * 60 / 2 / PI / WHEEL_RAD; //RAW to RPM
  
    dxl.setGoalVelocity(LEFT_ID, set_LRPM, UNIT_RPM);
    dxl.setGoalVelocity(RIGHT_ID, set_RRPM, UNIT_RPM);
  }
  finish = 1;
}

void coordinate()
{

  cnt_Lpos = dxl.getPresentPosition(LEFT_ID);
  cnt_Rpos = dxl.getPresentPosition(RIGHT_ID);
  L_RPM = dxl.getPresentVelocity(LEFT_ID,UNIT_RPM);
  R_RPM = dxl.getPresentVelocity(RIGHT_ID,UNIT_RPM);

  dif_Lpos = cnt_Lpos - prev_Lpos; //diff_l
  dif_Rpos = cnt_Rpos - prev_Rpos; //diff_r
  dis_L = dif_Lpos * thick_F; //dis_l
  dis_R = dif_Rpos * thick_F; //dis_r
  
  dis = (dis_L + dis_R) / 2; //dist
  encoder_yaw_new = atan2((dis_R - dis_L)/10000, WHEEL_WID); //theta
  
  encoder_yaw += encoder_yaw_new; //theta_
  if(encoder_yaw > 2*PI)
  {
    encoder_yaw -= 2*PI;
  }
  else if(encoder_yaw < (-2)*PI)
  {
    encoder_yaw += 2*PI;
  }
  
  encoder_yaw_q[0] = cos(0) * cos(0) * cos(encoder_yaw/2) + sin(0) * sin(0) * sin(encoder_yaw/2); //encoder_yaw's quetarnian
  encoder_yaw_q[1] = sin(0) * cos(0) * cos(encoder_yaw/2) - cos(0) * sin(0) * sin(encoder_yaw/2);
  encoder_yaw_q[2] = cos(0) * sin(0) * cos(encoder_yaw/2) + sin(0) * cos(0) * sin(encoder_yaw/2);
  encoder_yaw_q[3] = cos(0) * cos(0) * sin(encoder_yaw/2) - sin(0) * sin(0) * cos(encoder_yaw/2);

  dxl_x += dis * cos(encoder_yaw); //pose_x_
  dxl_y += dis * sin(encoder_yaw); //pose_y_

  prev_Lpos = cnt_Lpos;
  prev_Rpos = cnt_Rpos;
  

  //Publish
  yaw_msg.data = encoder_yaw*180/PI;
  
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_link";

  pose_msg.header.stamp = nh.now();
  pose_msg.header.frame_id = "map";

  pose_msg.pose.position.x = dxl_x/10000;
  pose_msg.pose.position.y = dxl_y/10000;
  pose_msg.pose.position.z = 0;

  pose_msg.pose.orientation.w = encoder_yaw_q[0];
  pose_msg.pose.orientation.x = encoder_yaw_q[1];
  pose_msg.pose.orientation.y = encoder_yaw_q[2];
  pose_msg.pose.orientation.z = -encoder_yaw_q[3];

  tfs_msg.header.stamp    = nh.now();
  tfs_msg.header.frame_id = "map";
  tfs_msg.child_frame_id  = "imu_link";

  tfs_msg.transform.rotation.w = pose_msg.pose.orientation.w;
  tfs_msg.transform.rotation.x = pose_msg.pose.orientation.x;
  tfs_msg.transform.rotation.y = pose_msg.pose.orientation.y;
  tfs_msg.transform.rotation.z = pose_msg.pose.orientation.z;

  tfs_msg.transform.translation.x = pose_msg.pose.position.x;
  tfs_msg.transform.translation.y = pose_msg.pose.position.y;
  tfs_msg.transform.translation.z = 0.0;
  imu_pub.publish(&imu_msg);

  pose_pub.publish(&pose_msg);
  yaw_pub.publish(&yaw_msg);

  tfbroadcaster.sendTransform(tfs_msg);
}
