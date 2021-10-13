#include <DynamixelWorkbench.h>

#define DEVICE_NAME ""
   

#define BAUDRATE  1000000
#define LEFT_ID    1
#define RIGHT_ID    2

DynamixelWorkbench dxl_wb;

void setup() 
{
  Serial.begin(57600);
  // while(!Serial); // Wait for Opening Serial Monitor

  const char *log;
  bool result = false;

  uint8_t left_id = LEFT_ID;
  uint8_t right_id = RIGHT_ID;
  uint16_t left_model = 0;
  uint16_t right_model = 0;

  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init");
  }
  else
  {
    Serial.print("Succeeded to init : ");
    Serial.println(BAUDRATE);  
  }

  result = dxl_wb.ping(left_id, &left_model, &log);
  result = dxl_wb.ping(right_id, &right_model, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to ping");
  }
  else
  {
    Serial.println("Succeeded to ping");
    Serial.print("left_id : ");
    Serial.println(left_id);
    Serial.print("right_id : ");
    Serial.println(right_id);
    Serial.print("left model_number : ");
    Serial.println(left_model);
    Serial.print("right model_number : ");
    Serial.println(right_model);
  }

  result = dxl_wb.wheelMode(left_id, 0, &log);
  result = dxl_wb.wheelMode(right_id, 0, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to change wheel mode");
  }
  else
  {
    Serial.println("Succeed to change wheel mode");
    Serial.println("Dynamixel is moving...");

    for (int count = 0; count < 3; count++)
    {
      dxl_wb.goalVelocity(left_id, (int32_t)-100);
      dxl_wb.goalVelocity(right_id, (int32_t)-100);
      delay(3000);

      dxl_wb.goalVelocity(left_id, (int32_t)100);
      dxl_wb.goalVelocity(right_id, (int32_t)100);
      delay(3000);
    }

    dxl_wb.goalVelocity(left_id, (int32_t)0);
    dxl_wb.goalVelocity(right_id, (int32_t)0);
  }
}

void loop() 
{

}
