#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
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

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(LEFT_ID);
  dxl.ping(RIGHT_ID);
  
  // Turn on the LED on DYNAMIXEL
  dxl.ledOn(LEFT_ID);
  dxl.ledOn(RIGHT_ID);
  delay(500);
  // Turn off the LED on DYNAMIXEL
  dxl.ledOff(LEFT_ID);
  dxl.ledOff(RIGHT_ID);
  delay(500);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(LEFT_ID);
  dxl.torqueOff(RIGHT_ID);
  dxl.setOperatingMode(LEFT_ID, OP_POSITION);
  dxl.setOperatingMode(RIGHT_ID, OP_POSITION);
  dxl.torqueOn(LEFT_ID);
  dxl.torqueOn(RIGHT_ID);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t key_input;
  DEBUG_SERIAL.println("Select Profile Type :");
  DEBUG_SERIAL.println("[1] Low Accel / High Vel");
  DEBUG_SERIAL.println("[2] Max Accel / Low Vel");
  DEBUG_SERIAL.println("[3] Max Accel / Max Vel");
  
  while(DEBUG_SERIAL.available()==0);
  key_input = DEBUG_SERIAL.read();

  switch(key_input) {
    case '1':
      // Low Profile Acceleration, High Profile Velocity
      // Refer to API documentation for available parameters
      // http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/#dynamixelshieldv010-or-above
      dxl.writeControlTableItem(PROFILE_ACCELERATION, LEFT_ID, 50);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, RIGHT_ID, 50);
      dxl.writeControlTableItem(PROFILE_VELOCITY, LEFT_ID, 300);
      dxl.writeControlTableItem(PROFILE_VELOCITY, RIGHT_ID, 300);
      break;
    case '2':
      // Max Profile Acceleration, Low Profile Velocity
      dxl.writeControlTableItem(PROFILE_ACCELERATION, LEFT_ID, 0);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, RIGHT_ID, 0);
      dxl.writeControlTableItem(PROFILE_VELOCITY, LEFT_ID, 100);
      dxl.writeControlTableItem(PROFILE_VELOCITY, RIGHT_ID, 100);
      break;
    case '3':
      // Max Profile Acceleration, Max Profile Velocity
      // WARNING : Please BE AWARE that this option will make a vibrant motion for PRO(A) or PRO+ series that requires high current supply.
      dxl.writeControlTableItem(PROFILE_ACCELERATION, LEFT_ID, 0);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, RIGHT_ID, 0);
      dxl.writeControlTableItem(PROFILE_VELOCITY, LEFT_ID, 0);
      dxl.writeControlTableItem(PROFILE_VELOCITY, RIGHT_ID, 0);
      break;
    default:
      break;
  }

  // Set Goal Position in RAW value
  dxl.setGoalPosition(LEFT_ID, 0);
  dxl.setGoalPosition(RIGHT_ID, 0);
  delay(500);
  // Check if DYNAMIXEL is in motion
  while(dxl.readControlTableItem(MOVING, LEFT_ID));
  while(dxl.readControlTableItem(MOVING, RIGHT_ID));
  
  // Set Goal Position in angle(degree)
  dxl.setGoalPosition(LEFT_ID, 179.0, UNIT_DEGREE);
  dxl.setGoalPosition(RIGHT_ID, 179.0, UNIT_DEGREE);
  delay(500);
  // Check if DYNAMIXEL is in motion
  while(dxl.readControlTableItem(MOVING, LEFT_ID));
  while(dxl.readControlTableItem(MOVING, RIGHT_ID));
}
