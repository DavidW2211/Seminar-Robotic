// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//       - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
// MPU6050 mpu;
MPU6050 mpu(0x69); // <-- use for AD0 high
MPU6050 mpuBase(0x68); // Second IMU

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// uncomment "OUTPUT_ANGULAR_ERROR" if you want output the angular error
// on the z axis between both used IMUs
#define OUTPUT_ANGULAR_ERROR

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define INTERRUPT_PIN_Base 3  // Second interrupt

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

uint16_t fifoCountBase;     // count of all bytes currently in FIFO
uint8_t fifoBufferBase[64]; // FIFO storage buffer
float eulerBase[3];         // [psi, theta, phi]    Euler angle container
Quaternion qBase;           // [w, x, y, z]         quaternion container

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[22] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0,0,0,0,0,0,0,0,0, 0x00, 0x00, '\r', '\n' };

bool firstLoop = true;
Quaternion offset;  //Initial rotational offset between both sensors


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

volatile bool mpuInterruptBase = false;     // indicates whether MPU Base interrupt pin has gone high
void dmpDataReadyBase() {
    mpuInterruptBase = true;
}

// ================================================================
// ===                     Dynamixel                            ===
// ================================================================
/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;
 
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

const int MAX_POSITION = 1800;
const int MIN_POSITION = 200;
const int START_POSITION = 1000;

int presentPos = 0;

const int ALPHA_THRESHOLD = 4;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    mpuBase.initialize();
    //pinMode(INTERRUPT_PIN, INPUT);
    //pinMode(INTERRUPT_PIN_Base, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(mpuBase.testConnection() ? F("Base connection successful") : F("Base connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        /* Disabled interrupt
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_Base), dmpDataReadyBase, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        */
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    devStatus = mpuBase.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpuBase.setXGyroOffset(220);
    mpuBase.setYGyroOffset(76);
    mpuBase.setZGyroOffset(-85);
    mpuBase.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpuBase.CalibrateAccel(6);
        mpuBase.CalibrateGyro(6);
        mpuBase.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpuBase.setDMPEnabled(true);

        /*
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpuBase.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        */
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpuBase.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

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
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 30);
    delay(2000);
    
    presentPos = dxl.getPresentPosition(DXL_ID);
    
    dxl.setGoalPosition(DXL_ID, START_POSITION);

    delay(8000);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything

    if (!dmpReady) return;
    // read a packet from FIFO

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer) && mpuBase.dmpGetCurrentFIFOPacket(fifoBufferBase)) { // Get the Latest packet  

          #ifdef OUTPUT_ANGULAR_ERROR
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpuBase.dmpGetQuaternion(&qBase, fifoBufferBase);

              /* If using a magnetometer and north reference
              if(firstLoop){
                firstLoop = false;
                offset = q.getDifferenceBetweenQuats(qBase);
                return;
              }*/
              
              //Quaternion diff = offset.getDifferenceBetweenQuats(q.getDifferenceBetweenQuats(qBase)); //If using a magnetometer and north reference
              Quaternion diff = q.getDifferenceBetweenQuats(qBase);

              Quaternion axisAngle = Quaternion(diff.w, -diff.x, diff.z, diff.y);

              VectorFloat up = VectorFloat(0,1,0);
              up.rotate(&axisAngle);

              float x = up.x;
              float y = up.y;

              VectorFloat adjustedUp = VectorFloat(x, y, 0);
              float angle = adjustedUp.getAngle(VectorFloat(0,1,0)) * 180/M_PI;

              if(x < 0){
                angle *= -1;
              }

              Serial.print("Angular error\t");
              Serial.println(angle);

              if(abs(angle) < ALPHA_THRESHOLD){
                return;
              }

              presentPos = dxl.getPresentPosition(DXL_ID);
              
              //Serial.println(String(presentPos));
              
              if(angle < 0){
                move(presentPos + 100);
              }
              else{
                move(presentPos - 100);
              }
                         
          #endif
       
          #ifdef OUTPUT_READABLE_EULER
              // display Euler angles in degrees
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetEuler(euler, &q);

              mpuBase.dmpGetQuaternion(&qBase, fifoBufferBase);
              mpuBase.dmpGetEuler(eulerBase, &qBase);
              
              Serial.print("Skin drift\t");
              Serial.print((eulerBase[0] - euler[0]) * 180/M_PI);
              Serial.print("\t");
              Serial.print((eulerBase[1] - euler[1]) * 180/M_PI);
              Serial.print("\t");
              Serial.println((eulerBase[2] - euler[2]) * 180/M_PI);
              
              
              /*Serial.print("Skin euler\t");
              Serial.print(euler[0] * 180/M_PI);
              Serial.print("\t");
              Serial.print(euler[1] * 180/M_PI);
              Serial.print("\t");
              Serial.println(euler[2] * 180/M_PI);
              *//*
              Serial.print("Base euler\t");
              Serial.print(eulerBase[0] * 180/M_PI);
              Serial.print("\t");
              Serial.print(eulerBase[1] * 180/M_PI);
              Serial.print("\t");
              Serial.println(eulerBase[2] * 180/M_PI);
              */
          #endif

          #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            
            teapotPacket[10] = fifoBufferBase[0];
            teapotPacket[11] = fifoBufferBase[1];
            teapotPacket[12] = fifoBufferBase[4];
            teapotPacket[13] = fifoBufferBase[5];
            teapotPacket[14] = fifoBufferBase[8];
            teapotPacket[15] = fifoBufferBase[9];
            teapotPacket[16] = fifoBufferBase[12];
            teapotPacket[17] = fifoBufferBase[13];
            Serial.write(teapotPacket, 22);
            teapotPacket[19]++; // packetCount, loops at 0xFF on purpose
        #endif
    }
}

void move(float newGoalPosition) {
  if (newGoalPosition > MAX_POSITION) {
    newGoalPosition = MAX_POSITION;
  } else if (newGoalPosition < MIN_POSITION) {
   newGoalPosition = MIN_POSITION;
  }
  dxl.setGoalPosition(DXL_ID, newGoalPosition);
}
