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
double thick_F = (WHEEL_RAD*2*PI)/(G*Re) *1000000; //1thick's distance

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

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

void twistMessageReceived(const geometry_msgs::Twist& msg)
{
  
  set_lin = msg.linear.x;
  set_ang = msg.angular.z;
  set_Lvel = set_lin + (set_ang * WHEEL_WID / 2); //target_speed_l
  set_Rvel = set_lin - (set_ang * WHEEL_WID / 2); //target_speed_r
  set_LRPM = set_Lvel * 60 / 2 / PI / WHEEL_RAD; //RAW to RPM
  set_RRPM = set_Rvel * 60 / 2 / PI / WHEEL_RAD; //RAW to RPM
  
  dxl.setGoalVelocity(LEFT_ID, set_LRPM, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, set_RRPM, UNIT_RPM);

}

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

void matrix_dot(double* C, double* A, double* B, int n, int l, int m)
{
  for(int i = 0; i < n; i++){
    for(int j = 0; j < m; j++){
      *(C+i*m+j) = 0;
      for(int k = 0; k < l; k++)
        *(C+i*m+j) += *(A + i*l + k) * (*(B + j + m*k));
    }
  }
}

int InverseMatrix3(double *m, double *mi)
{
  double det = m[0] * (m[4] * m[8] - m[7] * m[5]) -m[1] * (m[3] * m[8] - m[5] * m[6]) +m[2] * (m[3] * m[7] - m[4] * m[6]);
  if (det == 0) return 0;
  det = 1 / det;
  
  mi[0] = (m[4] * m[8] - m[7] * m[5]) * det;
  mi[1] = (m[2] * m[7] - m[1] * m[8]) * det;
  mi[2] = (m[1] * m[5] - m[2] * m[4]) * det;
  mi[3] = (m[5] * m[6] - m[3] * m[8]) * det;
  mi[4] = (m[0] * m[8] - m[2] * m[6]) * det;
  mi[5] = (m[3] * m[2] - m[0] * m[5]) * det;
  mi[6] = (m[3] * m[7] - m[6] * m[4]) * det;
  mi[7] = (m[6] * m[1] - m[0] * m[7]) * det;
  mi[8] = (m[0] * m[4] - m[3] * m[1]) * det;
  
  return 1;
}

int InverseMatrix4(double *A, double *A_I)
{
  double def;
  
  def = A[0]*A[5]*A[10]*A[15] + A[0]*A[6]*A[11]*A[13] + A[0]*A[7]*A[9]*A[14] 
  - A[0]*A[7]*A[10]*A[13] - A[0]*A[6]*A[9]*A[15] - A[0]*A[5]*A[11]*A[14] 
  - A[1]*A[4]*A[10]*A[15] - A[2]*A[4]*A[11]*A[13] - A[3]*A[4]*A[9]*A[14] 
  + A[3]*A[4]*A[10]*A[13] + A[2]*A[4]*A[9]*A[15] + A[1]*A[4]*A[11]*A[14] 
  + A[1]*A[6]*A[8]*A[15] + A[2]*A[7]*A[8]*A[13] + A[3]*A[5]*A[8]*A[14]
  - A[3]*A[6]*A[8]*A[13] - A[2]*A[5]*A[8]*A[15] - A[1]*A[7]*A[8]*A[14]
  - A[1]*A[6]*A[11]*A[12] - A[2]*A[7]*A[9]*A[12] - A[3]*A[5]*A[10]*A[12] 
  + A[3]*A[6]*A[9]*A[12] + A[2]*A[5]*A[11]*A[12] + A[1]*A[7]*A[10]*A[12];
  
  if (def==0)return 0;
  
  A_I[0] = {(A[5]*A[10]*A[15] + A[6]*A[11]*A[13] + A[7]*A[9]*A[14] - A[7]*A[10]*A[13] - A[6]*A[9]*A[15] - A[5]*A[11]*A[14])/def};
  A_I[1] = {((-1)*(A[1]*A[10]*A[15]) - A[2]*A[11]*A[13] - A[3]*A[9]*A[14] + A[3]*A[10]*A[13] + A[2]*A[9]*A[15] + A[1]*A[11]*A[14])/def};
  A_I[2] = {(A[1]*A[6]*A[15] + A[2]*A[7]*A[13] + A[3]*A[5]*A[14] - A[3]*A[6]*A[13] - A[2]*A[5]*A[15] - A[1]*A[7]*A[14])/def};
  A_I[3] = {((-1)*(A[1]*A[6]*A[11]) - A[2]*A[7]*A[9] - A[3]*A[5]*A[10] + A[3]*A[6]*A[9] + A[2]*A[5]*A[11] + A[1]*A[7]*A[10])/def};
  A_I[4] = {((-1)*(A[4]*A[10]*A[15]) - A[6]*A[11]*A[12] - A[7]*A[8]*A[14] + A[7]*A[10]*A[12] + A[6]*A[8]*A[15] + A[4]*A[11]*A[14])/def};
  A_I[5] = {(A[0]*A[10]*A[15] + A[2]*A[11]*A[12] + A[3]*A[8]*A[14] - A[3]*A[10]*A[12] - A[2]*A[8]*A[15] - A[0]*A[11]*A[14])/def};
  A_I[6] = {((-1)*(A[0]*A[6]*A[15]) - A[2]*A[7]*A[12] - A[3]*A[4]*A[14] + A[3]*A[6]*A[12] + A[2]*A[4]*A[15] + A[0]*A[7]*A[14])/def};
  A_I[7] = {(A[0]*A[6]*A[11] + A[2]*A[7]*A[8] + A[3]*A[4]*A[10] - A[3]*A[6]*A[8] - A[2]*A[4]*A[11] - A[0]*A[7]*A[10])/def};
  A_I[8] = {(A[4]*A[9]*A[15] + A[5]*A[11]*A[12] + A[7]*A[8]*A[13] - A[7]*A[9]*A[12] - A[5]*A[8]*A[15] - A[4]*A[11]*A[13])/def};
  A_I[9] = {((-1)*(A[0]*A[9]*A[15]) - A[1]*A[11]*A[12] - A[3]*A[8]*A[13] + A[3]*A[9]*A[12] + A[1]*A[8]*A[15] + A[0]*A[11]*A[13])/def};
  A_I[10] = {(A[0]*A[5]*A[15] + A[1]*A[7]*A[12] + A[3]*A[4]*A[13] - A[3]*A[5]*A[12] - A[1]*A[4]*A[15] - A[0]*A[7]*A[13])/def};
  A_I[11] = {((-1)*(A[0]*A[5]*A[11]) - A[1]*A[7]*A[8] - A[3]*A[4]*A[9] + A[3]*A[5]*A[8] + A[1]*A[4]*A[11] + A[0]*A[7]*A[9])/def};
  A_I[12] = {((-1)*(A[4]*A[9]*A[14]) - A[5]*A[10]*A[12] - A[6]*A[8]*A[13] + A[6]*A[9]*A[12] + A[5]*A[8]*A[14] + A[4]*A[10]*A[13])/def};
  A_I[13] = {(A[0]*A[9]*A[14] + A[1]*A[10]*A[12] + A[2]*A[8]*A[13] - A[2]*A[9]*A[12] - A[1]*A[8]*A[14] - A[0]*A[10]*A[13])/def};
  A_I[14] = {((-1)*(A[0]*A[5]*A[14]) - A[1]*A[6]*A[12] - A[2]*A[4]*A[13] + A[2]*A[5]*A[12] + A[1]*A[4]*A[14] + A[0]*A[6]*A[13])/def};
  A_I[15] = {(A[0]*A[5]*A[10] + A[1]*A[6]*A[8] + A[2]*A[4]*A[9] - A[2]*A[5]*A[8] - A[1]*A[4]*A[10] - A[0]*A[6]*A[9])/def};
  
  return 1;
}

void create_A(double* gyro, double* A)
{
  A[0] = 1;
  A[1] = -dt/2*gyro[0];
  A[2] = -dt/2*gyro[1];
  A[3] = -dt/2*gyro[2];
  A[4] = dt/2*gyro[0];
  A[5] = 1;
  A[6] = dt/2*gyro[2];
  A[7] = -dt/2*gyro[1];
  A[8] = dt/2*gyro[1];
  A[9] = -dt/2*gyro[2];
  A[10] = 1;
  A[11] = dt/2*gyro[0];
  A[12] = dt/2*gyro[2];
  A[13] = dt/2*gyro[1];
  A[14] = -dt/2*gyro[0];
  A[15] = 1;
}

void setup()
{
  Serial.begin(11520);

  nh.initNode();
  
  nh.subscribe(teleop_sub);

  nh.advertise(imu_pub);
  nh.advertise(pose_pub);
  nh.advertise(yaw_pub);
  tfbroadcaster.init(nh);

  mpu.begin();

  DEBUG_SERIAL.begin(11520);
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
  
  yaw = 0;
  posx[0] = 0;
  posy[0] = 0;
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
  
  dif_Lpos = cnt_Lpos - prev_Lpos; //diff_l
  dif_Rpos = cnt_Rpos - prev_Rpos; //diff_r
  
  dis_L = dif_Lpos * thick_F; //dis_l
  dis_R = dif_Rpos * thick_F; //dis_r
  dis = (dis_L + dis_R) / 2; //dist
  
  encoder_yaw_new = atan2((dis_L - dis_R)/1000000, WHEEL_WID); //theta
  encoder_yaw += encoder_yaw_new; //theta_
  
  encoder_yaw_q[0] = cos(0) * cos(0) * cos(encoder_yaw/2) + sin(0) * sin(0) * sin(encoder_yaw/2);
  encoder_yaw_q[1] = sin(0) * cos(0) * cos(encoder_yaw/2) - cos(0) * sin(0) * sin(encoder_yaw/2);
  encoder_yaw_q[2] = cos(0) * sin(0) * cos(encoder_yaw/2) + sin(0) * cos(0) * sin(encoder_yaw/2);
  encoder_yaw_q[3] = cos(0) * cos(0) * sin(encoder_yaw/2) - sin(0) * sin(0) * cos(encoder_yaw/2);

  if(encoder_yaw > 2*PI)
  {
    encoder_yaw -= 2*PI;
  }
  else if(encoder_yaw < (-2)*PI)
  {
    encoder_yaw += 2*PI;
  }
  
  dxl_x += (dis * cos(encoder_yaw))/1000000; //pose_x_
  dxl_y += (dis * sin(encoder_yaw))/1000000; //pose_y_

  prev_Lpos = cnt_Lpos;
  prev_Rpos = cnt_Rpos;

  KalmanFilter();
}

void KalmanFilter()
{
   
  mpu.gyro_get_adc();
  mpu.acc_get_adc();
  
  imu_msg.angular_velocity_covariance[0] = 1.2184696791468346e-07;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 1.2184696791468346e-07;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 1.2184696791468346e-07;
  imu_msg.linear_acceleration_covariance[0] = 8.999999999999999e-08;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 8.999999999999999e-08;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 8.999999999999999e-08;
  
  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;
  
  //sensor value
  gyro[0] = mpu.gyroADC[ROLL]*0.00106;
  gyro[1] = mpu.gyroADC[PITCH]*0.00106;
  gyro[2] = mpu.gyroADC[YAW]*0.00106; // *3.14/180/16.4 = *0.00106
  
  acc[0] = mpu.accADC[ROLL]*0.0006;
  acc[1] = mpu.accADC[PITCH]*0.0006;
  acc[2] = mpu.accADC[YAW]*0.0006; //*9.8/16384

  //Orientation Priori System Estimate : gyro
  create_A(gyro, A);
  matrix_dot(xp, A, x, 4, 4, 1);
  double Pp_a[16];
  matrix_dot(Pp_a, A, P, 4, 4, 4);
  double A_T[16] = {A[0], A[4], A[8], A[12], A[1], A[5], A[9], A[13], A[2], A[6], A[10], A[14], A[3], A[7], A[11], A[15]};
  double Pp_A[16];
  matrix_dot(Pp_A, Pp_a, A_T, 4, 4, 4);
  double Pp[16] = {Pp_A[0]+Q[0], Pp_A[1]+Q[1], Pp_A[2]+Q[2], Pp_A[3]+Q[3], Pp_A[4]+Q[4], Pp_A[5]+Q[5], Pp_A[6]+Q[6], Pp_A[7]+Q[7], Pp_A[8]+Q[8], Pp_A[9]+Q[9], Pp_A[10]+Q[10], Pp_A[11]+Q[11], Pp_A[12]+Q[12], Pp_A[13]+Q[13], Pp_A[14]+Q[14], Pp_A[15]+Q[15]};
  
  //Orientation Correction Stage1 : with acc
  H[0] = -2*x[2];
  H[1] = 2*x[3];
  H[2] = -2*x[0];
  H[3] = 2*x[1];
  H[4] = 2*x[1];
  H[5] = 2*x[0];
  H[6] = 2*x[3];
  H[7] = 2*x[2];
  H[8] = 2*x[0];
  H[9] = -2*x[1];
  H[10] = -2*x[2];
  H[11] = 2*x[3];
  
  h[0] = 2*x[1]*x[3] - 2*x[0]*x[2];
  h[1] = 2*x[0]*x[1] + 2*x[2]*x[3];
  h[2] = x[0]*x[0] - x[1]*x[1] - x[2]*x[2] + x[3]*x[3];
  h[0] = h[0]*9.8;
  h[1] = h[1]*9.8;
  h[2] = h[2]*9.8;
  
  z1[0] = acc[0];
  z1[1] = acc[1];
  z1[2] = acc[2];
  
  double H_T[12]={ H[0], H[4], H[8], H[1], H[5], H[9], H[2], H[6], H[10], H[3], H[7], H[11]};
  double Pp_h[12];

  matrix_dot(Pp_h, H, Pp, 3, 4, 4);

  double Pp_H[9];

  matrix_dot(Pp_H, Pp_h, H_T, 3, 4, 3);

  double R_v[9];

  matrix_dot(R_v, V, R, 3, 3, 3);

  double R_V[9];

  matrix_dot(R_V, R_v, V, 3, 3, 3);

  double temp[9] = {Pp_H[0]+R_V[0], Pp_H[1]+R_V[1], Pp_H[2]+R_V[2], Pp_H[3]+R_V[3], Pp_H[4]+R_V[4], Pp_H[5]+R_V[5], Pp_H[6]+R_V[6], Pp_H[7]+R_V[7], Pp_H[8]+R_V[8]};
  double inverse_temp[9];

  InverseMatrix3(temp, inverse_temp);

  double H_ti[12];

  matrix_dot(H_ti, Pp, H_T, 4, 4, 3);
  matrix_dot(K, H_ti, inverse_temp, 4, 3, 3);

  double z1_h[3] = {z1[0]-h[0], z1[1]-h[1], z1[2]-h[2]};
  double qe1[4];

  matrix_dot(qe1, K, z1_h, 4, 3, 1);

  qe1[3] = 0;

  double q1[4] ={ xp[0] + qe1[0], xp[1] + qe1[1], xp[2] + qe1[2], xp[3] + qe1[3]};
  double K_h[16];

  matrix_dot(K_h, K, H, 4, 3, 4);

  double I_Kh[16] = { 1 - K_h[0], -K_h[1], -K_h[2], -K_h[3], -K_h[4], 1 - K_h[5], -K_h[6], -K_h[7], -K_h[8], -K_h[9], 1 - K_h[10], -K_h[11], -K_h[12], -K_h[13], -K_h[14], 1 - K_h[15]};
  double P1[16];

  matrix_dot(P1, I_Kh, Pp, 4, 4, 4);
  
  //Orientation Correction Stage2 : with Encoder's yaw    
  z2[0] = encoder_yaw_q[0] - xp[0];
  z2[1] = encoder_yaw_q[1] - xp[1];
  z2[2] = encoder_yaw_q[2] - xp[2];
  z2[3] = encoder_yaw_q[3] - xp[3];
  
  double Pp_h2[16];
  double Pp_H2[12];

  matrix_dot(Pp_h2, H2, Pp, 4, 4, 4);
  matrix_dot(Pp_H2, Pp_h2, H2, 4, 4, 3);
  
  double temp2[16] = {Pp_H2[0]+1, Pp_H2[1], Pp_H2[2], Pp_H2[3], Pp_H2[4], Pp_H2[5]+1, Pp_H2[6], Pp_H2[7], Pp_H2[8], Pp_H2[9], Pp_H2[10]+1, Pp_H2[11], Pp_H2[12], Pp_H2[13], Pp_H2[14], Pp_H2[15]+1};
  double inverse_temp2[16];

  InverseMatrix4(temp2, inverse_temp2);

  double H_ti2[16];
  matrix_dot(H_ti2, Pp, H2, 4, 4, 4);
  matrix_dot(K2, H_ti2, inverse_temp2, 4, 4, 4);

  double qe2[4];
  matrix_dot(qe2, K2, z2, 4,4, 1);
  qe2[1] = 0;
  qe2[2] = 0;

  x[0] = q1[0] + qe2[0];
  x[1] = q1[1] + qe2[1];
  x[2] = q1[2] + qe2[2];
  x[3] = q1[3] + qe2[3];
  
  matrix_dot(K_h, K2, H2, 4, 4, 4);
  
  I_Kh[0] = 1 - K_h[0];
  I_Kh[1] = -K_h[1];
  I_Kh[2] = -K_h[2];
  I_Kh[3] = -K_h[3];
  I_Kh[4] = -K_h[4];
  I_Kh[5] = 1 - K_h[5];
  I_Kh[6] = -K_h[6];
  I_Kh[7] = -K_h[7];
  I_Kh[8] = -K_h[8];
  I_Kh[9] = -K_h[9];
  I_Kh[10] = 1 - K_h[10];
  I_Kh[11] = -K_h[11];
  I_Kh[12] = -K_h[12];
  I_Kh[13] = -K_h[13];
  I_Kh[14] = -K_h[14];
  I_Kh[15] = 1 - K_h[15];

  matrix_dot(P, I_Kh, P1, 4, 4, 4);

  double Q_abs = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
  quat[0] = x[0]/Q_abs;
  quat[1] = x[1]/Q_abs;
  quat[2] = x[2]/Q_abs;
  quat[3] = x[3]/Q_abs;
  
  //IMU position estimation
  double siny_cosp = 2 * (quat[0] * quat[3] + quat[1] * quat[2]);
  double cosy_cosp = 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]);
  
  yaw = atan2(siny_cosp, cosy_cosp); // yaw (z-axis rotation)
  local_acc_x = cos(yaw)*acc[0] - sin(yaw)*acc[1];
  local_acc_y = sin(yaw)*acc[0] + cos(yaw)*acc[1];

  //Position X Priori System Estimate : Encoder
  double A3[9] = {1, dt, dt*dt/2, 0 , 1, dt, 0, 0, 1};
  matrix_dot(pospx, A3, posx, 3, 3, 1);

  double Pp_a3[9];
  matrix_dot(Pp_a3, A3, P3x, 3, 3, 3);

  double A_T3[9] = {A3[0], A3[3], A3[6], A3[1], A3[4], A3[7], A3[2], A3[5], A3[8]};
  double Pp_A3[9];

  matrix_dot(Pp_A3, Pp_a3, A_T3, 3, 3, 3);

  double Pp3[9] = {Pp_A3[0]+Q3[0], Pp_A3[1]+Q3[1], Pp_A3[2]+Q3[2], Pp_A3[3]+Q3[3], Pp_A3[4]+Q3[4], Pp_A3[5]+Q3[5], Pp_A3[6]+Q3[6], Pp_A3[7]+Q3[7], Pp_A3[8]+Q3[8]};

  //Position X Correction Stage : with IMU position
  z3[0] = dxl_x - pospx[0];
  z3[1] = lin_Vel*cos(encoder_yaw) - pospx[1];
  z3[2] = local_acc_x - pospx[2];
  
  double Pp_h3[9];
  matrix_dot(Pp_h3, H3, Pp3, 3, 3, 3);

  double Pp_H3[3];
  matrix_dot(Pp_H3, Pp_h3, H3, 3, 3, 3);

  double R_v3[9];
  matrix_dot(R_v3, V, R, 3, 3, 3);

  double R_V3[9];
  matrix_dot(R_V3, R_v3, V, 3, 3, 3);

  double temp3[9] = {Pp_H3[0]+R_V3[0], Pp_H3[1]+R_V3[1], Pp_H3[2]+R_V3[2], Pp_H3[3]+R_V3[3], Pp_H3[4]+R_V3[4], Pp_H3[5]+R_V3[5], Pp_H3[6]+R_V3[6], Pp_H3[7]+R_V3[7], Pp_H3[8]+R_V3[8]};
  double inverse_temp3[9];

  InverseMatrix3(temp3, inverse_temp3);

  double H_ti3[9];

  matrix_dot(H_ti3, Pp3, H3, 3, 3, 3);
  matrix_dot(K3x, H_ti3, inverse_temp3, 3, 3, 3);

  double qe3[3];

  matrix_dot(qe3, K3x, z3, 3, 3, 1);
  posx[0] = pospx[0] + qe3[0];
  posx[1] = pospx[1] + qe3[1];
  posx[2] = pospx[2] + qe3[2];

  double K_h3[9];
  matrix_dot(K_h3, K3x, H3, 3, 3, 3);
  
  double I_Kh3[9] = { 1 - K_h3[0], -K_h3[1], -K_h3[2], -K_h3[3], 1-K_h3[4], -K_h3[5], -K_h3[6], -K_h3[7], 1-K_h3[8]};
  matrix_dot(P3x, I_Kh3, Pp3, 3, 3, 3);

  //Position Y Priori System Estimate : Encoder
  matrix_dot(pospy, A3, posy, 3, 3, 1);
  matrix_dot(Pp_a3, A3, P3y, 3, 3, 3);
  matrix_dot(Pp_A3, Pp_a3, A_T3, 3, 3, 3);

  Pp3[0] = Pp_A3[0]+Q3[0];
  Pp3[1] = Pp_A3[1]+Q3[1];
  Pp3[2] = Pp_A3[2]+Q3[2];
  Pp3[3] = Pp_A3[3]+Q3[3];
  Pp3[4] = Pp_A3[4]+Q3[4];
  Pp3[5] = Pp_A3[5]+Q3[5];
  Pp3[6] = Pp_A3[6]+Q3[6];
  Pp3[7] = Pp_A3[7]+Q3[7];
  Pp3[8] = Pp_A3[8]+Q3[8];
  
  //Position Y Correction Stage : with IMU position
  z4[0] = dxl_y - pospy[0];
  z4[1] = lin_Vel*sin(encoder_yaw) - pospy[1];
  z4[2] = local_acc_y - pospy[2];
  
  matrix_dot(Pp_h3, H3, Pp3, 3, 3, 3);
  matrix_dot(Pp_H3, Pp_h3, H3, 3, 3, 3);

  double temp4[9] = {Pp_H3[0]+R_V3[0], Pp_H3[1]+R_V3[1], Pp_H3[2]+R_V3[2], Pp_H3[3]+R_V3[3], Pp_H3[4]+R_V3[4], Pp_H3[5]+R_V3[5], Pp_H3[6]+R_V3[6], Pp_H3[7]+R_V3[7], Pp_H3[8]+R_V3[8]};
  double inverse_temp4[9];

  InverseMatrix3(temp4, inverse_temp4);

  matrix_dot(H_ti3, Pp3, H3, 3, 3, 3);
  matrix_dot(K3y, H_ti3, inverse_temp4, 3, 3, 3);
  matrix_dot(qe3, K3y, z4, 3, 3, 1);

  posy[0] = pospy[0] + qe3[0];
  posy[1] = pospy[1] + qe3[1];
  posy[2] = pospy[2] + qe3[2];

  matrix_dot(K_h3, K3y, H3, 3, 3, 3);

  I_Kh3[0] = 1 - K_h3[0];
  I_Kh3[1] = -K_h3[1];
  I_Kh3[2] = -K_h3[2];
  I_Kh3[9] = -K_h3[3];
  I_Kh3[9] = 1- K_h3[4];
  I_Kh3[9] = -K_h3[5];
  I_Kh3[9] = -K_h3[6];
  I_Kh3[9] = -K_h3[7];
  I_Kh3[9] = 1- K_h3[8];

  matrix_dot(P3y, I_Kh3, Pp3, 3, 3, 3);

  publish();
}

void publish()
{
  //Publish
  yaw_msg.data = yaw*180/PI;
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_link";
  
  imu_msg.orientation.w = quat[0];
  imu_msg.orientation.x = quat[1];
  imu_msg.orientation.y = quat[2];
  imu_msg.orientation.z = quat[3];
  
  imu_msg.angular_velocity.x = gyro[0];
  imu_msg.angular_velocity.y = gyro[1];
  imu_msg.angular_velocity.z = gyro[2];

  imu_msg.linear_acceleration.x = acc[0];
  imu_msg.linear_acceleration.y = acc[1];
  imu_msg.linear_acceleration.z = acc[2];

  pose_msg.header.stamp = nh.now();
  pose_msg.header.frame_id = "map";

  pose_msg.pose.position.x = posx[0];
  pose_msg.pose.position.y = posy[0];
  pose_msg.pose.position.z = 0;

  pose_msg.pose.orientation.w = quat[0];
  pose_msg.pose.orientation.x = quat[1];
  pose_msg.pose.orientation.y = quat[2];
  pose_msg.pose.orientation.z = quat[3];

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
