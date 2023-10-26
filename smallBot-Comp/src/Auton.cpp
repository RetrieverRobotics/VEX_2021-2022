#include "Auton.h"

#define TURN_SPEED 20

Pid turnPID(0.7, 0.002, 65, 5, TURN_SPEED, "Gyro");
// Pid rDrivePID(30, 0.002, 5, 5, 100, "Right Drive");
// Pid lDrivePID(30, 0.002, 5, 5, 100, "Left Drive");
// Pid rDrivePID(30, 0.002, 0, 5, 100, "Right Drive");
// Pid lDrivePID(30, 0.002, 0, 5, 100, "Left Drive");
Pid rDrivePID(37, 0.002, 0, 5, 100, "Right Drive");
Pid lDrivePID(37, 0.002, 0, 5, 100, "Left Drive");

void turnTo(double theta, double speed = TURN_SPEED, double lMult = 1, double rMult = 1) {
  double thresh = 0.2;

  turnPID.enable();
  // turnPID.setTarget(theta * 1); //Compensate for gyro drift
  turnPID.setTarget(theta * 1.03); //Compensate for gyro drift
  // turnPID.setTarget(theta * 1.1); //Compensate for gyro drift
  turnPID.maxOut = speed;

  while (gyroscope.rotation() > turnPID.getTarget() + thresh || gyroscope.rotation() < turnPID.getTarget() - thresh) {
    double rotateSpeed = turnPID.update(gyroscope.rotation());

    std::cout << "Gyro:   " << gyroscope.rotation() << std::endl;
    std::cout << "Target: " << turnPID.getTarget() << std::endl;
    std::cout << "Output: " << rotateSpeed << std::endl;

    RDrive_Group.spin(reverse, rotateSpeed * rMult, percentUnits::pct);
    LDrive_Group.spin(forward, rotateSpeed * lMult, percentUnits::pct);

    wait(20, msec);
  }

  std::cout << "Gyro:   " << gyroscope.rotation() << std::endl;

  // wait(250, msec);
  RDrive_Group.stop();
  LDrive_Group.stop();

  turnPID.disable();
}

void turn(double theta, int speed = TURN_SPEED, double lMult = 1, double rMult = 1) {
  turnTo(gyroscope.rotation() + theta, speed, lMult, rMult);
}

//5.82 revolutions to turn 360 degrees
// void turnTo(double theta, int speed = TURN_SPEED) {
//   double thresh = 0.2;

//   turnPID.enable();
//   // turnPID.setTarget(theta * 1); //Compensate for gyro drift
//   turnPID.setTarget(theta * 1.03); //Compensate for gyro drift
//   // turnPID.setTarget(theta * 1.1); //Compensate for gyro drift
//   turnPID.maxOut = speed;

//   while (gyroscope.rotation() > turnPID.getTarget() + thresh || gyroscope.rotation() < turnPID.getTarget() - thresh) {
//     double rotateSpeed = turnPID.update(gyroscope.rotation());

//     std::cout << "Gyro:   " << gyroscope.rotation() << std::endl;
//     std::cout << "Target: " << turnPID.getTarget() << std::endl;
//     std::cout << "Output: " << rotateSpeed << std::endl;

//     RDrive_Group.spin(reverse, rotateSpeed, percentUnits::pct);
//     LDrive_Group.spin(forward, rotateSpeed, percentUnits::pct);

//     wait(20, msec);
//   }

//   std::cout << "Gyro:   " << gyroscope.rotation() << std::endl;

//   // wait(250, msec);
//   RDrive_Group.stop();
//   LDrive_Group.stop();

//   turnPID.disable();
// }

void driveDistance(double distance, int speed = 50) {
  double thresh = 0.1;

  rDrivePID.enable();
  lDrivePID.enable();
  rDrivePID.setTarget(RDrive_Group.position(rotationUnits::rev) + 0.98 * distance / (WHEEL_CIRCUMFERENCE) * GEAR_RATIO); //3 is for a 1:3 gear ratio
  lDrivePID.setTarget(LDrive_Group.position(rotationUnits::rev) + 0.98 * distance / (WHEEL_CIRCUMFERENCE) * GEAR_RATIO);
  rDrivePID.maxOut = speed;
  lDrivePID.maxOut = speed;

  while ((RDrive_Group.position(rotationUnits::rev) > rDrivePID.getTarget() + thresh || RDrive_Group.position(rotationUnits::rev) < rDrivePID.getTarget() - thresh)
      && (LDrive_Group.position(rotationUnits::rev) > lDrivePID.getTarget() + thresh || LDrive_Group.position(rotationUnits::rev) < lDrivePID.getTarget() - thresh)) {
    double rSpeed = rDrivePID.update(RDrive_Group.position(rotationUnits::rev));
    double lSpeed = lDrivePID.update(LDrive_Group.position(rotationUnits::rev));

    std::cout << "Right Encoder:  " << RDrive_Group.position(rotationUnits::rev) << std::endl;
    std::cout << "Right Target:   " << rDrivePID.getTarget() << std::endl;
    std::cout << "Right Output:   " << rSpeed << std::endl;
    std::cout << "Left Encoder:   " << LDrive_Group.position(rotationUnits::rev) << std::endl;
    std::cout << "Left Target:    " << lDrivePID.getTarget() << std::endl;
    std::cout << "Left Output:    " << lSpeed << std::endl;

    RDrive_Group.spin(forward, rSpeed, percentUnits::pct);
    LDrive_Group.spin(forward, lSpeed, percentUnits::pct);

    wait(20, msec);
  }

  RDrive_Group.stop();
  LDrive_Group.stop();

  rDrivePID.disable();
  lDrivePID.disable();
}

void driveDistanceStraight(double distance, int speed = 50) {
  double thresh = 0.1;

  rDrivePID.enable();
  lDrivePID.enable();
  rDrivePID.setTarget(RDrive_Group.position(rotationUnits::rev) + 0.98 * distance / (WHEEL_CIRCUMFERENCE) * GEAR_RATIO);
  lDrivePID.setTarget(LDrive_Group.position(rotationUnits::rev) + 0.98 * distance / (WHEEL_CIRCUMFERENCE) * GEAR_RATIO);
  rDrivePID.maxOut = speed;
  lDrivePID.maxOut = speed;

  double rSpeed = 0;
  double lSpeed = 0;

  // bool reachedSpeed = false;

  RDrive_Group.spin(forward, rSpeed / 2, percentUnits::pct);
  LDrive_Group.spin(forward, lSpeed / 2, percentUnits::pct);
  wait (150, msec);

  while ((RDrive_Group.position(rotationUnits::rev) > rDrivePID.getTarget() + thresh || RDrive_Group.position(rotationUnits::rev) < rDrivePID.getTarget() - thresh)
      && (LDrive_Group.position(rotationUnits::rev) > lDrivePID.getTarget() + thresh || LDrive_Group.position(rotationUnits::rev) < lDrivePID.getTarget() - thresh)) {
    rSpeed = rDrivePID.update(RDrive_Group.position(rotationUnits::rev));
    lSpeed = lDrivePID.update(LDrive_Group.position(rotationUnits::rev));

    std::cout << "Right Encoder:  " << RDrive_Group.position(rotationUnits::rev) << std::endl;
    std::cout << "Right Target:   " << rDrivePID.getTarget() << std::endl;
    std::cout << "Right Output:   " << rSpeed << std::endl;
    std::cout << "Left Encoder:   " << LDrive_Group.position(rotationUnits::rev) << std::endl;
    std::cout << "Left Target:    " << lDrivePID.getTarget() << std::endl;
    std::cout << "Left Output:    " << lSpeed << std::endl;

    RDrive_Group.spin(forward, rSpeed, percentUnits::pct);
    LDrive_Group.spin(forward, lSpeed, percentUnits::pct);

    // if (!reachedSpeed) {
    //   wait (150, msec);
    //   rDrivePID.maxOut = speed;
    //   lDrivePID.maxOut = speed;
    //   reachedSpeed = true;
    // }

    wait(20, msec);
  }

  RDrive_Group.stop();
  LDrive_Group.stop();

  rDrivePID.disable();
  lDrivePID.disable();
}

// void ballsToTheWalls() {
//   double thresh = 0.1;

//   rDrivePID.enable();
//   lDrivePID.enable();
//   rDrivePID.setTarget(RDrive_Group.position(rotationUnits::rev) + 0.98 * distance / (WHEEL_CIRCUMFERENCE) * GEAR_RATIO);
//   lDrivePID.setTarget(LDrive_Group.position(rotationUnits::rev) + 0.98 * distance / (WHEEL_CIRCUMFERENCE) * GEAR_RATIO);
//   rDrivePID.maxOut = 100;
//   lDrivePID.maxOut = 100;

//   while ((RDrive_Group.position(rotationUnits::rev) > rDrivePID.getTarget() + thresh || RDrive_Group.position(rotationUnits::rev) < rDrivePID.getTarget() - thresh)
//       && (LDrive_Group.position(rotationUnits::rev) > lDrivePID.getTarget() + thresh || LDrive_Group.position(rotationUnits::rev) < lDrivePID.getTarget() - thresh)) {
//     double rSpeed = rDrivePID.update(RDrive_Group.position(rotationUnits::rev));
//     double lSpeed = lDrivePID.update(LDrive_Group.position(rotationUnits::rev));

//     std::cout << "Right Encoder:  " << RDrive_Group.position(rotationUnits::rev) << std::endl;
//     std::cout << "Right Target:   " << rDrivePID.getTarget() << std::endl;
//     std::cout << "Right Output:   " << rSpeed << std::endl;
//     std::cout << "Left Encoder:   " << LDrive_Group.position(rotationUnits::rev) << std::endl;
//     std::cout << "Left Target:    " << lDrivePID.getTarget() << std::endl;
//     std::cout << "Left Output:    " << lSpeed << std::endl;

//     RDrive_Group.spin(forward, rSpeed, percentUnits::pct);
//     LDrive_Group.spin(forward, lSpeed, percentUnits::pct);

//     wait(20, msec);
//   }

//   RDrive_Group.stop();
//   LDrive_Group.stop();

//   rDrivePID.disable();
//   lDrivePID.disable();
// }

void driveDistanceNoPID(double distance, int speed = 50, bool wait = true) {
  RDrive_Group.spinTo(RDrive_Group.position(rotationUnits::rev) + 0.98 * distance / (WHEEL_CIRCUMFERENCE) * GEAR_RATIO, rotationUnits::rev, speed, velocityUnits::pct, false);
  LDrive_Group.spinTo(LDrive_Group.position(rotationUnits::rev) + 0.98 * distance / (WHEEL_CIRCUMFERENCE) * GEAR_RATIO, rotationUnits::rev, speed, velocityUnits::pct, wait);

  // RDrive_Group.stop();
  // LDrive_Group.stop();
}

double getSpinAmount(double distance, rotationUnits unit) {
  double amount = 0;

  if (unit == rotationUnits::rev)
    amount = 0.98 * distance / (WHEEL_CIRCUMFERENCE) * GEAR_RATIO;
  else if (unit == rotationUnits::deg)
    amount = 0.98 * distance / (WHEEL_CIRCUMFERENCE) * GEAR_RATIO * 360;

  return amount;
}

double getCurDistance(motor_group group) {
  return group.position(rev) * WHEEL_CIRCUMFERENCE / (0.98 * GEAR_RATIO);
}

void testAuton() {
  LDrive_Group.resetPosition();
  RDrive_Group.resetPosition();
  driveDistanceStraight(48, 100);
  // driveDistanceNoPID(48, 100);

  // while (true) {
  //   std::cout << getCurDistance(LDrive_Group) << std::endl;
  //   wait(20, msec);
  // }

  // driveDistanceStraight(36, 100);
  // turnTo(180);
}

void matchAuton() {
  LDrive_Group.resetPosition();
  RDrive_Group.resetPosition();
  
  DriverControl::ArmPress();
  DriverControl::Claw4Press();
  // wait(1500, msec);
  // DriverControl::Claw4Press();
  // return;
  
  //Drive Forward, Lift 4bar, open claw
  // LDrive_Group.rotateFor(1000*GEAR_RATIO, degrees, 100, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(1000*GEAR_RATIO, degrees, 100, velocityUnits::pct);

  // driveDistance(45, 100);
  driveDistance(46, 100);
  // driveDistanceNoPID(45, 100, false);
  // wait(880, msec);

  // LDrive_Group.spin(fwd, 50, velocityUnits::pct);
  // RDrive_Group.spin(fwd, 50, velocityUnits::pct);

  // wait(250, msec);

  // LDrive_Group.spin(fwd, 100, velocityUnits::pct);
  // RDrive_Group.spin(fwd, 100, velocityUnits::pct);

  // // while (LDrive_Group.position(degrees) < 1000*GEAR_RATIO && RDrive_Group.position(degrees) < 1000*GEAR_RATIO) {
  // while (LDrive_Group.position(degrees) < getSpinAmount(12, degrees) && RDrive_Group.position(degrees) < getSpinAmount(12, degrees)) {
  //   std::cout << "Loop" << std::endl;
  //   wait(20, msec);
  // }
  // std::cout << LDrive_Group.position(degrees) << std::endl;
  // std::cout << RDrive_Group.position(degrees) << std::endl;

  // LDrive_Group.rotateTo(1300*GEAR_RATIO, degrees, 100, velocityUnits::pct, false);
  // RDrive_Group.rotateTo(1300*GEAR_RATIO, degrees, 100, velocityUnits::pct, false);

  // LDrive_Group.spin(fwd, 100, velocityUnits::pct);
  // RDrive_Group.spin(fwd, 100, velocityUnits::pct);
  // // LDrive_Group.rotateTo(1300*GEAR_RATIO, degrees, 100, velocityUnits::pct, false);
  // // RDrive_Group.rotateTo(1300*GEAR_RATIO, degrees, 100, velocityUnits::pct, false);

  // while (LDrive_Group.position(degrees) < 900*GEAR_RATIO && RDrive_Group.position(degrees) < 900*GEAR_RATIO)
  //   wait(20, msec);

  // // LDrive_Group.stop(brakeType::coast);
  // // RDrive_Group.stop(brakeType::coast);
  // LDrive_Group.spin(fwd, 20, velocityUnits::pct);
  // RDrive_Group.spin(fwd, 20, velocityUnits::pct);

  // wait(1, sec);
  // LDrive_Group.stop();
  // RDrive_Group.stop();

  // LDrive_Group.rotateTo(1300*GEAR_RATIO, degrees, 20, velocityUnits::pct, false);
  // RDrive_Group.rotateTo(1300*GEAR_RATIO, degrees, 20, velocityUnits::pct);

  // fourBar.rotateFor(5*3, degrees, 75, velocityUnits::pct, false);
  // // R4bar.rotateFor(5*3, degrees, 75, velocityUnits::pct, false);
  // // L4bar.rotateFor(5*3, degrees, 75, velocityUnits::pct, false);
  // Claw4.rotateTo(-100*3, degrees, 75, velocityUnits::pct);
  // Claw4.rotateTo(-100*3, degrees, 75, velocityUnits::pct);
  // wait(500, msec);

  // driveDistance(34, 100);

  // LDrive_Group.stop();
  // RDrive_Group.stop();
  // return;
  // wait(750, msec);

  //Drive forward slower
  // LDrive_Group.spinFor(450*GEAR_RATIO, degrees, 50, velocityUnits::pct, false);
  // RDrive_Group.spinFor(450*GEAR_RATIO, degrees, 50, velocityUnits::pct);
  // LDrive_Group.spinFor(getSpinAmount(30, deg), degrees, 50, velocityUnits::pct, false);
  // RDrive_Group.spinFor(getSpinAmount(30, deg), degrees, 50, velocityUnits::pct);
  // wait(250, msec);

  //Clamp claw on goal
  // Claw4.rotateTo(10*3, degrees, 75, velocityUnits::pct);
  // wait(500, msec);

  // DriverControl::ClawPress();
  DriverControl::Claw4Press();
  // wait(10, msec);
  // wait(500, msec);
  //Lock the claw by lifting four bar
  fourBar.spinFor(fwd, 15*7, deg, 100, velocityUnits::pct);

  //Drive Backwards towards plat
  // LDrive_Group.rotateFor(-1050*GEAR_RATIO, degrees, 80, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(-1050*GEAR_RATIO, degrees, 80, velocityUnits::pct);
  // LDrive_Group.rotateFor(-1600*GEAR_RATIO, degrees, 80, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(-1600*GEAR_RATIO, degrees, 80, velocityUnits::pct);
  // LDrive_Group.rotateFor(getSpinAmount(-44, deg), degrees, 80, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(getSpinAmount(-44, deg), degrees, 80, velocityUnits::pct);

  driveDistanceNoPID(-48, 80);
  wait(150, msec);

  //Wall reset
  // LDrive_Group.spin(reverse, 100, velocityUnits::pct);
  // RDrive_Group.spin(reverse, 100, velocityUnits::pct);
  // wait(2, sec);
  // LDrive_Group.stop();
  // RDrive_Group.stop();
  // gyroscope.resetAngle();
  // driveDistanceNoPID(2);

  // driveDistance(-42);

  //Turn for drop off
  // LDrive_Group.rotateFor(300*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(-300*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  // LDrive_Group.rotateFor(250*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(250*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  //Let go of goal
  // Claw4.rotateTo(-100*3, degrees, 75, velocityUnits::pct);
  // LDrive_Group.rotateFor(-250*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(-250*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  // LDrive_Group.rotateFor(-300*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(300*GEAR_RATIO, degrees, 40, velocityUnits::pct);

  // turn(-110);
  turnTo(-98);
  // turnTo(-90);

  // driveDistanceNoPID(-18, 30);
  driveDistanceNoPID(-16, 30);
  wait(250, msec);

  //Drive backwards towards plat
  // LDrive_Group.rotateFor(-350*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(-350*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  // LDrive_Group.rotateFor(-690*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(-690*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  //Turn towards goal on plat
  // LDrive_Group.rotateFor(-300*GEAR_RATIO, degrees, 50, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(300*GEAR_RATIO, degrees, 50, velocityUnits::pct);
  // //Drive into goal/ plat
  // LDrive_Group.rotateFor(-650*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(-650*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  // wait(500, msec);

  // driveDistance(-16);
  // turn(15, TURN_SPEED, -0.125, 1);

  //Grab goal with pneumatics
  // Claw.set(true);
  // wait(500, msec);
  DriverControl::ClawPress();
  wait(500, msec);
  // driveDistanceNoPID(12, 30);
  turnTo(-16, TURN_SPEED, 1, -0.33);
  // turnTo(0, TURN_SPEED, 1, -0.33);

  //Stop after we pull the goal off the platform
  // return;

  //Drive back and forth to grab rings from drivers
  turnTo(-180);
  DriverControl::ArmPress();
  Conveyor.spin(fwd, 100, velocityUnits::pct);
  // driveDistanceNoPID(6, 15);
  // while (true) {
  for (int i = 0; i < 8; i++) {
    driveDistanceNoPID(12, 15);
    driveDistanceNoPID(-12, 15);
  }

  // wait(1, sec);

  DriverControl::ArmPress();
  DriverControl::ClawPress();

  return;

  //Drive backwards towards side wall
  // LDrive_Group.rotateFor(500*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(500*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  // LDrive_Group.rotateFor(450*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(450*GEAR_RATIO, degrees, 40, velocityUnits::pct);

  // driveDistance(6);

  //Run conveyor to score preloads
  // Conveyor.rotateFor(5000, deg, 100, velocityUnits::pct);

  //Get the claw out of the way
  // DriverControl::ClawPress();
  // DriverControl::ArmPress();
  // Conveyor.spinFor(3, sec, 100, velocityUnits::pct);

  //Close 4barClaw
  // Claw4.rotateTo(0, degrees, 75, velocityUnits::pct);

  // DriverControl::ArmPress();

  //Drive Backwards for wall reset
  // TDrive_Group.spinFor(2000, msec, 40, velocityUnits::pct);

  // // driveDistance(4);
  // // turnTo(-10);
  // turnTo(-190);
  // return;
  // TDrive_Group.spinFor(2, sec, 40, velocityUnits::pct);
  // TDrive_Group.stop();
  // gyroscope.setRotation(0.5, rotationUnits::rev);
  // // gyroscope.resetAngle();
  // wait(100, msec);

  //Drive Off Wall Slightly
  // LDrive_Group.rotateFor(-200*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // RDrive_Group.rotateFor(-200*GEAR_RATIO, degrees, 40, velocityUnits::pct);

  // driveDistance(-12);

  // //Turn 90 to have 4bar facing opposing side
  // // LDrive_Group.rotateFor(-300*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // // RDrive_Group.rotateFor(300*GEAR_RATIO, degrees, 40, velocityUnits::pct);

  // turnTo(0);

  // //Drive Towards Opposing Side
  // // LDrive_Group.rotateFor(-900*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // // RDrive_Group.rotateFor(-900*GEAR_RATIO, degrees, 40, velocityUnits::pct);

  driveDistanceNoPID(24);

  // //Turn 90 to face rings
  // // LDrive_Group.rotateFor(-280*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  // // RDrive_Group.rotateFor(280*GEAR_RATIO, degrees, 40, velocityUnits::pct,false);
  // // fourBar.rotateFor(100*3, degrees, 75, velocityUnits::pct);
  // // // R4bar.rotateFor(100*3, degrees, 75, velocityUnits::pct, false);
  // // // L4bar.rotateFor(100*3, degrees, 75, velocityUnits::pct);
  // // wait(500, msec);

  turnTo(70);

  // //Drive forward and collect Rings
  // // Conveyor.rotateFor(10000, deg, 100, velocityUnits::pct,false);
  // // LDrive_Group.rotateFor(1200*GEAR_RATIO, degrees, 30, velocityUnits::pct, false);
  // // RDrive_Group.rotateFor(1200*GEAR_RATIO, degrees, 30, velocityUnits::pct);
  // // wait(2000, msec);

  DriverControl::ArmPress();
  wait(150, msec);
  Conveyor.spin(fwd, 100, velocityUnits::pct);
  driveDistanceNoPID(50, 30);
  wait(2000, msec);

  // //Drop Goal
  // // Claw.set(false);
  // // wait(500, msec);
  Conveyor.stop();
  DriverControl::armsLight();
  // DriverControl::ClawPress();
  // driveDistance(6);
}

void BenMatch() {
  LDrive_Group.resetPosition();
  RDrive_Group.resetPosition();

  //Drive Forward, Lift 4bar, open claw
  LDrive_Group.spinFor(1300*GEAR_RATIO, degrees, 100, velocityUnits::pct, false);
  RDrive_Group.spinFor(1300*GEAR_RATIO, degrees, 100, velocityUnits::pct);
  // fourBar.rotateFor(5*3, degrees, 75, velocityUnits::pct, false);
  // // R4bar.rotateFor(5*3, degrees, 75, velocityUnits::pct, false);
  // // L4bar.rotateFor(5*3, degrees, 75, velocityUnits::pct, false);
  // Claw4.rotateTo(100*3, degrees, 75, velocityUnits::pct);
  // Claw4.rotateTo(100*3, degrees, 75, velocityUnits::pct);
  // DriverControl::Claw4Press();
  wait(500, msec);

  //Drive forward slower
  LDrive_Group.spinFor(125*GEAR_RATIO, degrees, 50, velocityUnits::pct, false);
  RDrive_Group.spinFor(125*GEAR_RATIO, degrees, 50, velocityUnits::pct);
  wait(250, msec);

  //Clamp claw on goal
  Claw4.spinTo(10*3, degrees, 75, velocityUnits::pct);
  wait(500, msec);

  //Drive Backwards towards plat
  LDrive_Group.spinFor(-1050*GEAR_RATIO, degrees, 80, velocityUnits::pct, false);
  RDrive_Group.spinFor(-1050*GEAR_RATIO, degrees, 80, velocityUnits::pct);

  //Turn for drop off
  LDrive_Group.spinFor(300*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(-300*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  LDrive_Group.spinFor(250*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(250*GEAR_RATIO, degrees, 40, velocityUnits::pct);

  //Let go of goal
  Claw4.spinTo(-100*3, degrees, 75, velocityUnits::pct);
  LDrive_Group.spinFor(-250*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(-250*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  LDrive_Group.spinFor(-300*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(300*GEAR_RATIO, degrees, 40, velocityUnits::pct);

  //Drive backwards towards plat
  LDrive_Group.spinFor(-350*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(-350*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  //Turn towards goal on plat
  LDrive_Group.spinFor(-300*GEAR_RATIO, degrees, 50, velocityUnits::pct, false);
  RDrive_Group.spinFor(300*GEAR_RATIO, degrees, 50, velocityUnits::pct);
  //Drive into goal/ plat
  LDrive_Group.spinFor(-650*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(-650*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  wait(500, msec);

  //Grab goal with pneumatics
  Claw.set(true);
  wait(500, msec);

  //Drive backwards towards side wall
  LDrive_Group.spinFor(500*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(500*GEAR_RATIO, degrees, 40, velocityUnits::pct);

  //Run conveyor to score preloads
  Conveyor.spinFor(5000, deg, 100, velocityUnits::pct);

  //Close 4barClaw
  Claw4.spinTo(0, degrees, 75, velocityUnits::pct);

  //Drive Backwards for wall reset
  TDrive_Group.spinFor(2000, msec, 40, velocityUnits::pct);

  //Drive Off Wall Slightly
  LDrive_Group.spinFor(-200*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(-200*GEAR_RATIO, degrees, 40, velocityUnits::pct);

  //Turn 90 to have 4bar facing opposing side
  LDrive_Group.spinFor(-300*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(300*GEAR_RATIO, degrees, 40, velocityUnits::pct);

  //Drive Towards Opposing Side
  LDrive_Group.spinFor(-900*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(-900*GEAR_RATIO, degrees, 40, velocityUnits::pct);

  //Turn 90 to face rings
  LDrive_Group.spinFor(-280*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(280*GEAR_RATIO, degrees, 40, velocityUnits::pct,false);
  fourBar.spinFor(100*3, degrees, 75, velocityUnits::pct);
  // R4bar.rotateFor(100*3, degrees, 75, velocityUnits::pct, false);
  // L4bar.rotateFor(100*3, degrees, 75, velocityUnits::pct);
  wait(500, msec);

  //Drive forward and collect Rings
  Conveyor.spinFor(10000, deg, 100, velocityUnits::pct,false);
  LDrive_Group.spinFor(1200*GEAR_RATIO, degrees, 30, velocityUnits::pct, false);
  RDrive_Group.spinFor(1200*GEAR_RATIO, degrees, 30, velocityUnits::pct);
  wait(2000, msec);

  //Drop Goal
  Claw.set(false);
  wait(500, msec);
}

void skillsAuton() {
  wait(8000, msec);
  //Drive into goal
  LDrive_Group.spinFor(-500*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(-500*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  wait(500, msec);
  //Pickup GOal
  Claw.set(true);
  wait(500, msec);
  //Drive Back
  LDrive_Group.spinFor(470*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(470*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  //Turn 90
  LDrive_Group.spinFor(305*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(-305*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  //Drive Towards Yellow Goal
  LDrive_Group.spinFor(1400*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(1400*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  fourBar.spinFor(5*3, degrees, 75, velocityUnits::pct, false);
  // R4bar.rotateFor(5*3, degrees, 75, velocityUnits::pct, false);
  // L4bar.rotateFor(5*3, degrees, 75, velocityUnits::pct, false);
  Claw4.spinTo(-100*3, degrees, 75, velocityUnits::pct);
  Claw4.spinTo(-100*3, degrees, 75, velocityUnits::pct);
  wait(1000, msec);
  //Close Claw grab goal
  Claw4.spinTo(10*3, degrees, 75, velocityUnits::pct);
  wait(500, msec);
  //Lift 4bar
  fourBar.spinFor(50*7, degrees, 75, velocityUnits::pct);
  // R4bar.rotateFor(50*7, degrees, 75, velocityUnits::pct, false);
  // L4bar.rotateFor(50*7, degrees, 75, velocityUnits::pct);
  //Drive Forward 
  LDrive_Group.spinFor(800*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(800*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  //Turn 90
  LDrive_Group.spinFor(270*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(-270*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  //Drive Forward 
  LDrive_Group.spinFor(900*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(900*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  wait(500, msec);
  //Turn 90
  LDrive_Group.spinFor(-320*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(320*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  wait(500, msec);
  //Lift 4bar
  fourBar.spinFor(70*7, degrees, 75, velocityUnits::pct);
  // R4bar.rotateFor(70*7, degrees, 75, velocityUnits::pct, false);
  // L4bar.rotateFor(70*7, degrees, 75, velocityUnits::pct);
  //Drive Forward 
  TDrive_Group.spinFor(2000, msec, 40, velocityUnits::pct);
  wait(1000, msec);
  //Drop goal/ open claw
  Claw4.spinFor(-100*3, degrees, 75, velocityUnits::pct);
  wait(500,msec);
  //Drive Back fromm plat
  LDrive_Group.spinFor(-350*GEAR_RATIO, degrees, 50, velocityUnits::pct, false);
  RDrive_Group.spinFor(-350*GEAR_RATIO, degrees, 50, velocityUnits::pct);
  //Turn Right
  //Turn 90
  LDrive_Group.spinFor(-305*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(305*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  wait(500, msec);
  //Drop Pneumatic Goal
  Claw.set(false);
  wait(500, msec);
  //Drive forward 
  LDrive_Group.spinFor(1000*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(1000*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  fourBar.spinFor(-120*7, degrees, 75, velocityUnits::pct);
  // R4bar.rotateFor(-120*7, degrees, 75, velocityUnits::pct, false);
  // L4bar.rotateFor(-120*7, degrees, 75, velocityUnits::pct);
  wait(2000, msec);
  //Drive forward slow
  LDrive_Group.spinFor(200*GEAR_RATIO, degrees, 20, velocityUnits::pct, false);
  RDrive_Group.spinFor(200*GEAR_RATIO, degrees, 20, velocityUnits::pct);
  wait(1000, msec);
  //Close Claw/grab goal
  Claw4.spinTo(10*3, degrees, 75, velocityUnits::pct);
  wait(500, msec);
  //Backup
  LDrive_Group.spinFor(-500*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(-500*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  //Turn towards us
  LDrive_Group.spinFor(-305*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(305*GEAR_RATIO, degrees, 40, velocityUnits::pct);
  //Drive towards us
  LDrive_Group.spinFor(1500*GEAR_RATIO, degrees, 40, velocityUnits::pct, false);
  RDrive_Group.spinFor(1500*GEAR_RATIO, degrees, 40, velocityUnits::pct);

// //Lower Lift
// R4bar.rotateFor(-100*3, degrees, 75, velocityUnits::pct, false);
// L4bar.rotateFor(-100*3, degrees, 75, velocityUnits::pct);
// //Let Go of Goal
// Claw4.rotateTo(-100*3, degrees, 75, velocityUnits::pct);
// wait(500, msec);
}
