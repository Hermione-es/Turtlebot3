#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
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
double gyro[3];
double acc[3];
double mag[3];
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

double wheel_wid = 0.16; //turtlebot3_burger's width
double wheel_rad = 0.033; //turtlebot3_burger's radius
double G = 1; // turtlebot3 gear ratio (XL430-W250)
double Re = 4096; // turtlebot3 encoder's resolution (XL430-W250)
double thick_F = (wheel_rad*2*PI)/(G*Re) *10000; //1thick's distance

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

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
sensor_msgs::MagneticField mag_msg;
tf::TransformBroadcaster tfbroadcaster;
std_msgs::Float64 yaw_msg;
geometry_msgs::PoseStamped pose_msg;
geometry_msgs::Vector3 lin_msg;
geometry_msgs::Vector3 ang_msg;
geometry_msgs::TransformStamped tfs_msg;

ros::Publisher imu_pub("encoder_orientation", &imu_msg);
ros::Publisher pose_pub("pose", &pose_msg);
ros::Publisher yaw_pub("yaw", &yaw_msg);
ros::Publisher lin_pub("lin_vel", &lin_msg);
ros::Publisher ang_pub("ang_vel", &ang_msg);
ros::Publisher mag_pub("magnetic_field", &mag_msg);

ros::Subscriber<geometry_msgs::Twist> teleop_sub("cmd_vel",&twistMessageReceived);


void setup()
{
  Serial.begin(115200);

  nh.initNode();
  
  nh.advertise(imu_pub);
  nh.advertise(pose_pub);
  nh.advertise(yaw_pub);
  nh.advertise(lin_pub);
  nh.advertise(ang_pub);
  nh.advertise(mag_pub);

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
  mpu.gyro_get_adc();
  mpu.acc_get_adc();
  mpu.mag_get_adc();

  cnt_Lpos = dxl.getPresentPosition(LEFT_ID);
  cnt_Rpos = dxl.getPresentPosition(RIGHT_ID);
  L_RPM = dxl.getPresentVelocity(LEFT_ID,UNIT_RPM);
  R_RPM = dxl.getPresentVelocity(RIGHT_ID,UNIT_RPM);

  L_Vel = 2 * PI * wheel_rad * L_RPM / 60; //left wheel welocity
  R_Vel = 2 * PI * wheel_rad * R_RPM / 60; //right wheel velocity

  lin_Vel = (L_Vel + R_Vel) /2; //linear_velocity
  ang_Vel = (R_Vel - L_Vel) /0.16; //angular_velocity

  dif_Lpos = cnt_Lpos - prev_Lpos; //diff_l
  dif_Rpos = cnt_Rpos - prev_Rpos; //diff_r
  dis_L = dif_Lpos * thick_F; //dis_l
  dis_R = dif_Rpos * thick_F; //dis_r
  
  dis = (dis_L + dis_R) / 2; //dist
  encoder_yaw_new = atan2((dis_R - dis_L)/10000, wheel_wid); //theta
  
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

  //sensor value
  gyro[0] = mpu.gyroADC[ROLL]*0.0010642;
  gyro[1] = mpu.gyroADC[PITCH]*0.0010642;
  gyro[2] = mpu.gyroADC[YAW]*0.0010642; // *3.14/180/16.4 = *0.00106
  
  acc[0] = mpu.accADC[ROLL]*0.000598550415;
  acc[1] = mpu.accADC[PITCH]*0.000598550415;
  acc[2] = mpu.accADC[YAW]*0.000598550415; //*9.8/16384 = *0.0006

  mag[0] = mpu.magADC[ROLL]*(15e-8);
  mag[1] = mpu.magADC[PITCH]*(15e-8);
  mag[2] = mpu.magADC[YAW]*(15e-8);
  
  //Publish
  yaw_msg.data = encoder_yaw*180/PI;

  lin_msg.x = lin_Vel;
  ang_msg.z = ang_Vel;
  
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_link";
  
  imu_msg.orientation.w = encoder_yaw_q[0];
  imu_msg.orientation.x = encoder_yaw_q[1];
  imu_msg.orientation.y = encoder_yaw_q[2];
  imu_msg.orientation.z = -encoder_yaw_q[3];
  
  imu_msg.angular_velocity.x = gyro[0];
  imu_msg.angular_velocity.y = gyro[1];
  imu_msg.angular_velocity.z = gyro[2];

  imu_msg.linear_acceleration.x = acc[0];
  imu_msg.linear_acceleration.y = acc[1];
  imu_msg.linear_acceleration.z = acc[2];
  
  mag_msg.header.stamp    = nh.now();
  mag_msg.header.frame_id = "mag_link";
  
  mag_msg.magnetic_field.x = mag[0];
  mag_msg.magnetic_field.y = mag[1];
  mag_msg.magnetic_field.z = mag[2];

  pose_msg.header.stamp = nh.now();
  pose_msg.header.frame_id = "map";

  pose_msg.pose.position.x = dxl_x/10000;
  pose_msg.pose.position.y = -dxl_y/10000;
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
  mag_pub.publish(&mag_msg);
  lin_pub.publish(&lin_msg);
  ang_pub.publish(&ang_msg);
  pose_pub.publish(&pose_msg);
  yaw_pub.publish(&yaw_msg);

  tfbroadcaster.sendTransform(tfs_msg);
}
