#include "encoder_cal_config.h"
const uint8_t DXL_DIR_PIN = 84;
const uint8_t RIGHT_ID = 1;
const uint8_t LEFT_ID = 2;
const double DXL_PROTOCOL_VERSION = 2.0;

cMPU9250 mpu;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;


/*******************************************************************************
* Setup function
*******************************************************************************/
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

  //Initialize
  _init_();
  _initCov_();

}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  
  if(millis() < 30000)
  {
    if (millis()-pre_time >= PERIOD)
    {
      pre_time = millis();
      coordinate();
      publishMsg();
    }
  }

  else
  {
    backOrigin();
    if (millis()-pre_time >= PERIOD)
    {
      pre_time = millis();
      coordinate();
      publishMsg();
    }

    if (finish == 1) 
    {
      exit(0);
    }
  }
  nh.spinOnce();
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
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

/*******************************************************************************
* Publish msgs (IMU data, Pose data, Yaw data, tf)
*******************************************************************************/
void publishMsg()
{
  yaw_msg.data = encoder_yaw*180/PI/10000;
  
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
  pose_msg.pose.orientation.z = encoder_yaw_q[3];

  tfs_msg.header.stamp    = nh.now();
  tfs_msg.header.frame_id = "map";
  tfs_msg.child_frame_id  = "imu_link";

  tfs_msg.transform.rotation.w = -pose_msg.pose.orientation.w;
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

/*******************************************************************************
* Back to origin
*******************************************************************************/
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

/*******************************************************************************
* Calculate the coordinate
*******************************************************************************/
void coordinate()
{

  cnt_Lpos = dxl.getPresentPosition(LEFT_ID);
  cnt_Rpos = dxl.getPresentPosition(RIGHT_ID);
  L_RPM = dxl.getPresentVelocity(LEFT_ID,UNIT_RPM);
  R_RPM = dxl.getPresentVelocity(RIGHT_ID,UNIT_RPM);

  dif_Lpos = cnt_Lpos - prev_Lpos; //diff_l
  dif_Rpos = cnt_Rpos - prev_Rpos; //diff_r
  dis_L = dif_Lpos * THICK_F_10000; //dis_l
  dis_R = dif_Rpos * THICK_F_10000; //dis_r
  
  dis = (dis_L + dis_R) / 2; //dist
  encoder_yaw_new = atan2(dis_R - dis_L, WHEEL_WID); //theta
  
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
  
  
} 

/*******************************************************************************
* Initialization
*******************************************************************************/
void _init_()
{
  prev_Lpos = dxl.getPresentPosition(LEFT_ID);
  prev_Rpos = dxl.getPresentPosition(RIGHT_ID);
  
  encoder_yaw = 0;
  dxl_x = 0;
  dxl_y = 0;
}

void _initCov_()
{
  
}
