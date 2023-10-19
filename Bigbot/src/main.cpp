#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// RMotor1              motor         16              
// RMotor2              motor         17              
// RMotor3              motor         18              
// RMotor4              motor         19              
// RMotor5              motor         20              
// LMotor1              motor         11              
// LMotor2              motor         12              
// LMotor3              motor         13              
// LMotor4              motor         14              
// LMotor5              motor         15              
// claw                 motor_group   3, 4            
// piston               digital_out   A               
// gyroscope            gyro          B               
// fourBar              motor_group   9, 10           
// conveyor             motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "driverControl.h"
#include "Auton.h"

using namespace vex;

//Dead ports:
//2, 6, 8, 9, 10, 12, 14

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
motor_group LDrive_Group(LMotor1, LMotor2, LMotor3, LMotor4, LMotor5);
motor_group RDrive_Group(RMotor1, RMotor2, RMotor3, RMotor4, RMotor5);

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
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  LDrive_Group.setStopping(brake);
  RDrive_Group.setStopping(brake);
  claw.setStopping(brake);
  // fourBar.setStopping(brake);

  gyroscope.calibrate();
  while (gyroscope.isCalibrating());
  gyroscope.resetAngle();
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
  // testAuton();
  // skillsAuton();
  matchAuton();
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
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    DriverControl driverCtl = DriverControl();
    driverCtl.Run();

    wait(20, msec); // Sleep the task for a short amount of time to prevent wasted resources.
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
