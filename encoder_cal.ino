#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
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
double L_Vel;
double R_Vel;
double L_RPM;
double R_RPM;
double lin_Vel;
double ang_Vel;
double encoder_yaw = 0;
double encoder_yaw_q[4];
double encoder_yaw_new = 0;
double cnt_Lpos;
double cnt_Rpos;
double prev_Lpos;
double prev_Rpos;
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

double wheel_wid = 0.16; //turtlebot3_burger's width
double wheel_rad = 0.033; //turtlebot3_burger's radius
double G = 1; // turtlebot3 gear ratio (XL430-W250)
double Re = 4096; // turtlebot3 encoder's resolution (XL430-W250)
double thick_F = (wheel_rad*2*PI)/(G*Re) *10000;
double thick_deg = 360 / Re; //[deg/pulse]
double thick_rad = thick_deg * PI / 180; //deg2rad

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

double gyro[3];
double acc[3];
double mag[3];
double quat[4];
double posx[3] = {0, 0, 0};
double posy[3] = {0, 0, 0};
double pospx[3];
double pospy[3];
double H[12];
double H2[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
double H3[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
double Q[16] = {1e-6, 0, 0, 0, 0, 1e-6, 0, 0, 0, 0, 1e-6, 0, 0, 0, 0, 1e-6};
double Q3[9] = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};
double R[9] = {2, 0, 0, 0, 2, 0, 0, 0, 2};
double V[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
double P[16] = {0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001};
double P3x[9];
double P3y[9];
double A[16];
double K[12];
double K2[16];
double K3x[9];
double K3y[9];
double x[4] = {1, 0, 0, 0};
double xp[4];
double z1[3];
double z2[4];
double z3[3];
double z4[3];
double h[3];

double local_acc_x = 0;
double local_acc_y = 0;

void twistMessageReceived(const geometry_msgs::Twist& msg)
{
  
  set_lin = msg.linear.x;
  set_ang = msg.angular.z;
  set_Lvel = set_lin + (set_ang * wheel_wid / 2); //target_speed_l
  set_Rvel = set_lin - (set_ang * wheel_wid / 2); //target_speed_r
  set_LRPM = set_Lvel * 60 / 2 / PI / wheel_rad; //RAW to RPM
  set_RRPM = set_Rvel * 60 / 2 / PI / wheel_rad; //RAW to RPM
  
  dxl.setGoalVelocity(LEFT_ID, set_LRPM, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, set_RRPM, UNIT_RPM);

}

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
geometry_msgs::PoseStamped pose_msg;
ros::Publisher imu_pub("kalamn_orientation", &imu_msg);
ros::Publisher pose_pub("pose", &pose_msg);
geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;
ros::Subscriber<geometry_msgs::Twist> teleop_sub("cmd_vel",&twistMessageReceived);


void setup()
{
  Serial.begin(115200);

  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(pose_pub);
  tfbroadcaster.init(nh);

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
  nh.subscribe(teleop_sub);
  
  prev_Lpos = dxl.getPresentPosition(LEFT_ID);
  prev_Rpos = dxl.getPresentPosition(RIGHT_ID);

  dif_Lpos = prev_Lpos;
  dif_Rpos = prev_Rpos;
  dis_L = dif_Lpos * thick_F /10000;
  dis_R = dif_Rpos * thick_F /10000;
  dis = (dis_L + dis_R) / 2;
  
  encoder_yaw = (dis_R - dis_L) / wheel_wid;
  dxl_x = dis*cos(encoder_yaw);
  dxl_y = dis*sin(encoder_yaw);
}

void loop()
{

  static uint32_t pre_time;
  if (millis()-pre_time >= PERIOD)
  {
    pre_time = millis();
    coordinate();
  }
  nh.spinOnce();
}

void coordinate()
{
  cnt_Lpos = dxl.getPresentPosition(LEFT_ID);
  cnt_Rpos = dxl.getPresentPosition(RIGHT_ID);
  L_RPM = dxl.getPresentVelocity(LEFT_ID,UNIT_RPM);
  R_RPM = dxl.getPresentVelocity(RIGHT_ID,UNIT_RPM);
  L_Vel = 2 * PI * wheel_rad * L_RPM / 60; //RPM to RAW
  R_Vel = 2 * PI * wheel_rad * R_RPM / 60; //RPM to RAW
  lin_Vel = (L_Vel + R_Vel) / 2; //linear_velocity
  ang_Vel = (R_Vel - L_Vel) / wheel_wid; //angular_velocity

  dif_Lpos = cnt_Lpos - prev_Lpos;
  dif_Rpos = cnt_Rpos - prev_Rpos;
  dis_L = dif_Lpos * thick_F /10000; //diff_l
  dis_R = dif_Rpos * thick_F /10000; //diff_r
  
  dis = (dis_L + dis_R) / 2; //dist
  encoder_yaw_new = (dis_R - dis_L) / wheel_wid; //theta
  
  encoder_yaw += encoder_yaw_new; //theta_
  if(encoder_yaw > 2*PI)
  {
    encoder_yaw -= 2*PI;
  }
  else if(encoder_yaw < (-2)*PI)
  {
    encoder_yaw += 2*PI;
  }
  
  encoder_yaw_q[0] = cos(0) * cos(0) * cos(encoder_yaw/2) + sin(0) * sin(0) * sin(encoder_yaw/2);
  encoder_yaw_q[1] = sin(0) * cos(0) * cos(encoder_yaw/2) - cos(0) * sin(0) * sin(encoder_yaw/2);
  encoder_yaw_q[2] = cos(0) * sin(0) * cos(encoder_yaw/2) + sin(0) * cos(0) * sin(encoder_yaw/2);
  encoder_yaw_q[3] = cos(0) * cos(0) * sin(encoder_yaw/2) - sin(0) * sin(0) * cos(encoder_yaw/2);

  dxl_x += dis * cos(encoder_yaw_new); //pose_x_
  dxl_y += dis * sin(encoder_yaw_new); //pose_y_
  dxl_theta = encoder_yaw;

  prev_Lpos = cnt_Lpos;
  prev_Rpos = cnt_Rpos;
  
  
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;
  
  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;
  

  //Publish
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_link";

  pose_msg.header.stamp = nh.now();
  pose_msg.header.frame_id = "map";

  pose_msg.pose.position.x = dxl_x;
  pose_msg.pose.position.y = dxl_y;
  pose_msg.pose.position.z = 0;

  pose_msg.pose.orientation.w = encoder_yaw_q[0];
  pose_msg.pose.orientation.x = encoder_yaw_q[1];
  pose_msg.pose.orientation.y = encoder_yaw_q[2];
  pose_msg.pose.orientation.z = encoder_yaw_q[3];

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

  tfbroadcaster.sendTransform(tfs_msg);
}
