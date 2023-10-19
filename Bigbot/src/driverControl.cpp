#include "driverControl.h"
#include <iostream>

bool DriverControl::clawToggle = false;

void DriverControl::clawPress() {
  claw.rotateTo((clawToggle ? 0 : 85) * 7 / 3, degrees, false);
  clawToggle = !clawToggle;
}

void DriverControl::clawUp() {
  claw.rotateFor(forward, 2 * 7/3, deg, false);
}

void DriverControl::clawDown() {
  claw.rotateFor(forward, -2 * 7/3, deg, false);
}

void DriverControl::clawReset() {
  claw.rotateTo(0, deg, false);
}

bool DriverControl::fourBarToggle = false;

void DriverControl::fourBarPress() {
  fourBar.rotateTo((fourBarToggle ? 0 : 100) * 7, degrees, 100, velocityUnits::pct, false);
  fourBarToggle = !fourBarToggle;
}

bool DriverControl::pistonToggle = false;

void DriverControl::pistonPress() {
  pistonToggle = !pistonToggle;
  piston.set(pistonToggle);
}

bool speedMult = false;

void ToggleSpeed() {
  speedMult = !speedMult;
}

void DriverControl::Run() {
  //R1 intake
  //R2 outtake
  //L1 toggle 4bar
  //L2 toggle pneumatic
  //A mogo
  //Up and down mogo adjust

  Controller1.ButtonA.pressed(clawPress);
  Controller1.ButtonL1.pressed(fourBarPress);
  Controller1.ButtonL2.pressed(pistonPress);
  Controller1.ButtonUp.pressed(clawReset);
  Controller1.ButtonUp.pressed(clawUp);
  Controller1.ButtonDown.pressed(clawDown);
 
  int loopTimeMs = 20;

  // gyroscope.calibrate();
  // gyro2.calibrate();
  // while (gyroscope.isCalibrating() || gyro2.isCalibrating());
  // gyroscope.resetAngle();
  // gyro2.resetAngle();

  while (true) {
    // std::cout << "Claw:    " << claw.position(degrees) << std::endl;
    // std::cout << "Four Bar:   " << fourBar.position(degrees) << std::endl;
    std::cout << "Hello" << std::endl;
    // std::cout << "Gyro:       " << gyroscope.rotation() << std::endl;
    // std::cout << "Right:      " << RDrive_Group.position(rotationUnits::rev) << std::endl;
    // std::cout << "Left:       " << LDrive_Group.position(rotationUnits::rev) << std::endl;

    // std::cout << "MPU:        " << mpu.value(analogUnits::range8bit) << std::endl;
    // std::cout << "MPU:        " << mpu.value(percentUnits::pct) << std::endl;

    // std::cout << "Left Power: " << Controller1.Axis3.value() + Controller1.Axis1.value() << std::endl;
    // std::cout << "Right Power: " << Controller1.Axis3.value() - Controller1.Axis1.value() << std::endl;

    LDrive_Group.spin(forward, (Controller1.Axis3.value() + Controller1.Axis1.value())/* * (speedMult ? 0.20 : 1)*/, pct); 
    RDrive_Group.spin(forward, (Controller1.Axis3.value() - Controller1.Axis1.value())/* * (speedMult ? 0.20 : 1)*/, pct);

    //Conveyor
    if (Controller1.ButtonR1.pressing())
      conveyor.spin(fwd, CONVEYOR_SPEED, pct);
    else if (Controller1.ButtonR2.pressing())
      conveyor.spin(reverse, CONVEYOR_SPEED, pct);
    else
      conveyor.stop();

    wait(loopTimeMs, msec);
  }
}
