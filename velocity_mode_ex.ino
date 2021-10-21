  
#include <Dynamixel2Arduino.h>


#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 84;// OpenCR Board's DIR PIN.    
 

const uint8_t LEFT_ID = 1;
const uint8_t RIGHT_ID = 2;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(LEFT_ID);
  dxl.ping(RIGHT_ID);


  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(LEFT_ID);
  dxl.torqueOff(RIGHT_ID);
  dxl.setOperatingMode(LEFT_ID, OP_POSITION);
  dxl.setOperatingMode(RIGHT_ID, OP_POSITION);
  dxl.torqueOn(LEFT_ID);
  dxl.torqueOn(RIGHT_ID);

  DEBUG_SERIAL.println("Initalize");
  dxl.setGoalVelocity(LEFT_ID, 60);
  dxl.setGoalVelocity(RIGHT_ID, 60);

  dxl.setGoalPosition(LEFT_ID, 0);
  dxl.setGoalPosition(RIGHT_ID, 0);

  delay(2000);
  
  dxl.setGoalVelocity(LEFT_ID, 140);
  dxl.setGoalVelocity(RIGHT_ID, 140);
  dxl.setGoalPosition(LEFT_ID, 4000);
  dxl.setGoalPosition(RIGHT_ID, 4000);  
  DEBUG_SERIAL.print("Present Velocity : ");
  DEBUG_SERIAL.print(dxl.getPresentVelocity(LEFT_ID));
  DEBUG_SERIAL.print("  ,  ");
  DEBUG_SERIAL.println(dxl.getPresentVelocity(RIGHT_ID));

  delay(2000);

  DEBUG_SERIAL.println("Initalize");
  dxl.setGoalVelocity(LEFT_ID, 60);
  dxl.setGoalVelocity(RIGHT_ID, 60);

  dxl.setGoalPosition(LEFT_ID, 0);
  dxl.setGoalPosition(RIGHT_ID, 0);



}

void loop() {

  // Set Goal Velocity using RPM



}
