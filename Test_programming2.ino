#include <DynamixelWorkbench.h>

#define BAUDRATE            57600
#define BAUDRATE_TO_DXL     1000000
#define LEFT_ID            1
#define RIGHT_ID            2
 
DynamixelWorkbench dxl_wb;https://github.com/Hermione-es/Turtlebot3

void setup() {
    // put your setup code here, to run once:
    Serial.begin(BAUDRATE);
    while(!Serial); // Open a Serial Monitor

    dxl_wb.begin("", BAUDRATE_TO_DXL);
    dxl_wb.ping(LEFT_ID);
    dxl_wb.ping(RIGHT_ID);

    dxl_wb.jointMode(LEFT_ID);
    dxl_wb.jointMode(RIGHT_ID);
}

void loop() {
    // put your main code here, to run repeatedly:
    dxl_wb.goalPosition(LEFT_ID, 0);
    dxl_wb.goalPosition(RIGHT_ID, 0);
    delay(2000);
    dxl_wb.goalPosition(LEFT_ID, 2000);
    dxl_wb.goalPosition(RIGHT_ID, 2000);
    delay(2000);
}
