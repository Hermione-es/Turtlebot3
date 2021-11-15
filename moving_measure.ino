#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 84; 
const uint8_t RIGHT_ID = 1;
const uint8_t LEFT_ID = 2;
const float DXL_PROTOCOL_VERSION = 2.0;
HardwareTimer Timer(TIMER_CH1);

float L_Vel;
float R_Vel;
float L_RPM;
float R_RPM;
float lin_Vel;
float ang_Vel;
float present_Lpos;
float present_Rpos;
float new_Lpos;
float new_Rpos;
float dif_Lpos;
float dif_Rpos;
float dis_L;
float dis_R;
float dis = 0;
float deg = 0;
float new_x;
float new_y;
float x = 0;
float y = 0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;   

void setup() {
  
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
  dxl.setGoalVelocity(LEFT_ID, 0);
  dxl.setGoalVelocity(RIGHT_ID, 0);
  
  new_Lpos = dxl.getPresentPosition(LEFT_ID);
  new_Rpos = dxl.getPresentPosition(RIGHT_ID);

  
  Timer.stop();
  Timer.setPeriod(1000000);
  Timer.attachInterrupt(coordinate);
  Timer.start();

}

void loop() {

  
}

void coordinate()
{
    
  present_Lpos = dxl.getPresentPosition(LEFT_ID);
  present_Rpos = dxl.getPresentPosition(RIGHT_ID);

  L_RPM = dxl.getPresentVelocity(LEFT_ID,UNIT_RPM);
  R_RPM = dxl.getPresentVelocity(RIGHT_ID,UNIT_RPM);

  L_Vel = 2 * 3.141592 * 0.066 * L_RPM / 60;
  R_Vel = 2 * 3.141592 * 0.066 * R_RPM / 60;
  
  lin_Vel = (L_Vel + R_Vel) /2;
  ang_Vel = (R_Vel - L_Vel) /0.16 /0.066 ;
  
  DEBUG_SERIAL.print("Robot's Linear Velocity(m/s) : ");
  DEBUG_SERIAL.print(lin_Vel);
  DEBUG_SERIAL.print("  ,  ");
  DEBUG_SERIAL.print("Robot's Angualr Velocity(rad/s) : ");
  DEBUG_SERIAL.println(ang_Vel);
  
  dif_Lpos = present_Lpos - new_Lpos;
  dif_Rpos = present_Rpos - new_Rpos;
      
  dis_L = dif_Lpos * 0.00002 * 2.54;
  dis_R = dif_Rpos * 0.00002 * 2.54;
  
  dis = (dis_L + dis_R) / 2;
  deg = ((dis_R - dis_L) / 160) * 57.2958 * 100;

  new_x = sin(deg) * dis;
  new_y = cos(deg) * dis;

  x += new_x;
  y += new_y;
      
  DEBUG_SERIAL.print("x-coordinate : ");
  DEBUG_SERIAL.print(x);
  DEBUG_SERIAL.print("  ,  ");
  DEBUG_SERIAL.print("y-coordinate : ");
  DEBUG_SERIAL.println(y);

  new_Lpos = dxl.getPresentPosition(LEFT_ID);
  new_Rpos = dxl.getPresentPosition(RIGHT_ID);



}
