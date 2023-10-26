#include "driverControl.h"

bool DriverControl::ArmToggle = true;

void DriverControl::ArmPress(){
  // fourBar.rotateTo(ArmToggle ? 12*7 : 120*7, degrees, 75, velocityUnits::pct, false);
  fourBar.spinTo(ArmToggle ? 11*7 : 120*7, degrees, 75, velocityUnits::pct, false);
  ArmToggle = !ArmToggle;
}

void DriverControl::armsLight() {
  // fourBar.rotateTo(12*7, degrees, 75, velocityUnits::pct, false);
  fourBar.spinTo(11*7, degrees, 75, velocityUnits::pct, false);
}

void DriverControl::armDown() {
  fourBar.spinTo(0, degrees, 75, velocityUnits::pct, false);
}

bool DriverControl::ClawToggle = true;

void DriverControl::ClawPress() {
  Claw.set(ClawToggle);
  ClawToggle = !ClawToggle;
}

bool DriverControl::Claw4Toggle = false;

void DriverControl::Claw4Press() {
  // Claw4.rotateTo(Claw4Toggle ? -6*3 : 100*3, deg, 100, velocityUnits::pct, false);
  Claw4.spinTo(Claw4Toggle ? -7*3 : 100*3, deg, 100, velocityUnits::pct, false);
  Claw4Toggle = !Claw4Toggle;
}

void claw4Close() {
  Claw4.spinFor(-2*7, deg, 100, velocityUnits::pct, false);
}

void DriverControl::Run() {
  Controller1.ButtonA.pressed(ClawPress);
  Controller1.ButtonL1.pressed(ArmPress);
  Controller1.ButtonDown.pressed(armDown);
  Controller1.ButtonL2.pressed(Claw4Press);
  Controller1.ButtonY.pressed(claw4Close);

  //bool b = true;
  int loopTimeMs = 20;

  // GyroB.calibrate();
  // GyroB.startCalibration();
  // while (GyroB.isCalibrating()) wait(20, msec);

  // task pidTask(DrivePidTask);
  // DrivePidEnabled = false;

  while (1) {
    // std::cout << GyroB.value(rotationUnits::deg) << std::endl;
    // std::cout << GyroB.angle() << std::endl;
    // std::cout << GyroB.rotation() << std::endl;
    // std::cout << GyroB.heading() << std::endl;
    // std::cout << std::endl;

    //gyro testing
    // if(Controller1.ButtonX.pressing()){
    //   DrivePidEnabled = true;
    //   drivePid.SetTarget(90);
    // }
    // else{
    //  DrivePidEnabled = false;
    // }

    //  std::cout << L2Bar.position(deg) << std::endl;

    LDrive_Group.spin(forward, (Controller1.Axis3.value() + Controller1.Axis1.value() * 0.8), pct);
    RDrive_Group.spin(forward, (Controller1.Axis3.value() - Controller1.Axis1.value() * 0.8), pct);

    if (Controller1.ButtonR1.pressing()) {
      Conveyor.spin(reverse, CONVEYOR_SPEED, pct);
    } else if (Controller1.ButtonR2.pressing()) {
      Conveyor.spin(fwd, CONVEYOR_SPEED, pct);
    } else {
      Conveyor.stop();
    }

    wait(loopTimeMs, msec);
  }
}
