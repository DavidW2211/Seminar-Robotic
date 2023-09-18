#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;
 
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

const int START_POSITION = 1000;

int presentPos = 0;


void setup() {
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
    dxl.begin(57600);
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    // Get DYNAMIXEL information
    dxl.ping(DXL_ID);
    
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_POSITION);
    dxl.torqueOn(DXL_ID);
    
    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 10);
    delay(2000);
    
    presentPos = dxl.getPresentPosition(DXL_ID);
    
    dxl.setGoalPosition(DXL_ID, START_POSITION);

}
void loop() {

}
