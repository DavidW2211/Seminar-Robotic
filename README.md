#Preamble

Repository with the content of the "Wearable Robotics Seminar" conducted at UdS. Note that this repo contains only the files that you need to build our prototype, but no step-by-step explanation. 
For more detailed information, please read the attached paper about our work.

## 3D-Prints

The stl. files for 3D printing the prototype parts

## RoboticSeminarVisualization

A demo created in Unity-3D that visualizes the concept and processing of the prototype. One can either clone the project and edit it in Unity-3D or execute ./Builds/RoboticSeminarVisualization.exe.

##Sketches

The sketches and plugins for Arduino:
- DualSensor: The main sketch that runs on the Arduino Uno/OpenRB-150. It also contains a processing sketch for visualization of the sensors
- DynamixelWizard: This must be uploaded if one wants to use the dynamixel wizard to control the *weight motor*
- I2C_Scanner: Scan the correct I2C-Addresses of your board, to double-check that *DualSensor* uses the right ones
- MotorConfiguration: Run this before you add the motor to the prototype. It ensures that the motor rotation is correct
- MPU6050: This is **very important**. It is a library that must be placed in the *libraries* folder of Arduino. Otherwise, nothing works!
