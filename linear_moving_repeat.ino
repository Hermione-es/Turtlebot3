#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 84; 
const uint8_t LEFT_ID = 1;
const uint8_t RIGHT_ID = 2;
const float DXL_PROTOCOL_VERSION = 2.0;
unsigned long start_time;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

float present_Lpos = 0;
float present_Rpos = 0;
float new_Lpos = 0;
float new_Rpos = 0;
float dif_Lpos = 0;
float dif_Rpos = 0;
float dis_L = 0;
float dis_R = 0;
float new_dis = 0;
float total_dis = 0;

void setup() {

  DEBUG_SERIAL.begin(115200);
  
  dxl.begin(1000000);
  
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(LEFT_ID);
  dxl.ping(RIGHT_ID);

  dxl.torqueOff(LEFT_ID);
  dxl.torqueOff(RIGHT_ID);
  dxl.setOperatingMode(LEFT_ID, OP_VELOCITY);
  dxl.setOperatingMode(RIGHT_ID, OP_VELOCITY);
  dxl.torqueOn(LEFT_ID);
  dxl.torqueOn(RIGHT_ID);
  start_time = 0;
  dxl.setGoalVelocity(LEFT_ID, 5);
  dxl.setGoalVelocity(RIGHT_ID, 5);
  
  delay(1000);

}

void loop() {
  start_time = micros();

  if(start_time % 2000000 ==0)
  {
    present_Lpos = dxl.getPresentPosition(LEFT_ID);
    present_Rpos = dxl.getPresentPosition(RIGHT_ID);
    
    DEBUG_SERIAL.print("Present Velocity(raw) : ");
    DEBUG_SERIAL.print(dxl.getPresentVelocity(LEFT_ID));
    DEBUG_SERIAL.print(" , ");
    DEBUG_SERIAL.println(dxl.getPresentVelocity(RIGHT_ID));  
    new_Lpos = dxl.getPresentPosition(LEFT_ID);
    new_Rpos = dxl.getPresentPosition(RIGHT_ID);
    dif_Lpos = new_Lpos - present_Lpos;
    dif_Rpos = new_Rpos - present_Rpos;
      
    dis_L = dif_Lpos * 0.0002;
    dis_R = dif_Rpos * 0.0002;
    new_dis = (dis_L + dis_R) / 2;
    total_dis += new_dis;

    DEBUG_SERIAL.print("Total distance : ");
    DEBUG_SERIAL.println(total_dis);
   }
}
