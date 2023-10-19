/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// RMotor1              motor         1               
// RMotor2              motor         2               
// RMotor3              motor         3               
// RMotor4              motor         4               
// LMotor4              motor         9               
// LMotor1              motor         6               
// LMotor2              motor         7               
// LMotor3              motor         8               
// Conveyor             motor         11              
// Claw                 digital_out   A               
// gyroscope            gyro          B               
// fourBar              motor_group   13, 12          
// Claw4                motor         5               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
// #include "driverControl.h"
#include "Auton.h"

using namespace vex;

motor_group LDrive_Group = motor_group(LMotor1, LMotor2, LMotor3, LMotor4);
motor_group RDrive_Group = motor_group(RMotor1, RMotor2, RMotor3, RMotor4);
motor_group TDrive_Group = motor_group(RMotor1, RMotor2, RMotor3, RMotor4, LMotor1, LMotor2, LMotor3, LMotor4);

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  //Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  gyroscope.calibrate();
  while (gyroscope.isCalibrating());

  // fourBar.resetPosition();
  // LDrive_Group.resetPosition();
  // RDrive_Group.resetPosition();
  // Claw4.resetPosition();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
   gyroscope.resetAngle();
  //  testAuton();
  //  skillsAuton();
   matchAuton();
  //  BenMatch();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
  // old code
  // TestControl test = TestControl();
  // test.Run();

  //new driverControl code
  DriverControl driverCtl = DriverControl();
  driverCtl.Run();
  // Run();


  // User control code here, inside the loop
  while (1) {
    wait(20, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
