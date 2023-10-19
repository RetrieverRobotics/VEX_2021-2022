#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor RMotor1 = motor(PORT1, ratio6_1, true);
motor RMotor2 = motor(PORT2, ratio6_1, true);
motor RMotor3 = motor(PORT3, ratio6_1, true);
motor RMotor4 = motor(PORT4, ratio6_1, false);
motor LMotor4 = motor(PORT9, ratio6_1, true);
motor LMotor1 = motor(PORT6, ratio6_1, false);
motor LMotor2 = motor(PORT7, ratio6_1, false);
motor LMotor3 = motor(PORT8, ratio6_1, false);
motor Conveyor = motor(PORT11, ratio18_1, false);
digital_out Claw = digital_out(Brain.ThreeWirePort.A);
gyro gyroscope = gyro(Brain.ThreeWirePort.B);
motor fourBarMotorA = motor(PORT13, ratio18_1, false);
motor fourBarMotorB = motor(PORT12, ratio18_1, true);
motor_group fourBar = motor_group(fourBarMotorA, fourBarMotorB);
motor Claw4 = motor(PORT5, ratio18_1, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}