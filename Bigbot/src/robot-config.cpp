#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor RMotor1 = motor(PORT16, ratio6_1, true);
motor RMotor2 = motor(PORT17, ratio6_1, false);
motor RMotor3 = motor(PORT18, ratio6_1, true);
motor RMotor4 = motor(PORT19, ratio6_1, true);
motor RMotor5 = motor(PORT20, ratio6_1, false);
motor LMotor1 = motor(PORT11, ratio6_1, true);
motor LMotor2 = motor(PORT12, ratio6_1, false);
motor LMotor3 = motor(PORT13, ratio6_1, false);
motor LMotor4 = motor(PORT14, ratio6_1, false);
motor LMotor5 = motor(PORT15, ratio6_1, true);
motor clawMotorA = motor(PORT3, ratio36_1, false);
motor clawMotorB = motor(PORT4, ratio36_1, true);
motor_group claw = motor_group(clawMotorA, clawMotorB);
digital_out piston = digital_out(Brain.ThreeWirePort.A);
gyro gyroscope = gyro(Brain.ThreeWirePort.B);
motor fourBarMotorA = motor(PORT9, ratio18_1, true);
motor fourBarMotorB = motor(PORT10, ratio18_1, false);
motor_group fourBar = motor_group(fourBarMotorA, fourBarMotorB);
motor conveyor = motor(PORT8, ratio18_1, true);

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