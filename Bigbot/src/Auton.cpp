#include "Auton.h"

#define TURN_SPEED 20

Pid turnPID(0.7, 0.002, 65, 5, TURN_SPEED, "Gyro");
// Pid turnPID(0.7, 0.002, 75, 5, TURN_SPEED, "Gyro");
Pid rDrivePID(25, 0.002, 5, 5, 100, "Right Drive");
Pid lDrivePID(25, 0.002, 5, 5, 100, "Left Drive");

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
}

void driveDistanceNoPID(double distance, int speed = 50) {
  RDrive_Group.rotateTo(RDrive_Group.position(rotationUnits::rev) + 0.98 * distance / (WHEEL_CIRCUMFERENCE) * GEAR_RATIO, rotationUnits::rev, speed, velocityUnits::pct, false);
  LDrive_Group.rotateTo(LDrive_Group.position(rotationUnits::rev) + 0.98 * distance / (WHEEL_CIRCUMFERENCE) * GEAR_RATIO, rotationUnits::rev, speed, velocityUnits::pct);

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

void ballsToTheWalls(double time, directionType dir) {
  RDrive_Group.spin(dir, 15, pct);
  LDrive_Group.spin(dir, 15, pct);
  wait(100, msec);

  RDrive_Group.spin(dir, 25, pct);
  LDrive_Group.spin(dir, 25, pct);
  wait(100, msec);
  
  RDrive_Group.spin(dir, 50, pct);
  LDrive_Group.spin(dir, 50, pct);
  wait(100, msec);
  
  RDrive_Group.spin(dir, 100, pct);
  LDrive_Group.spin(dir, 100, pct);

  wait(time - 525, msec);
  
  RDrive_Group.spin(dir, 50, pct);
  LDrive_Group.spin(dir, 50, pct);
  wait(150, msec);

  DriverControl::pistonPress();

  RDrive_Group.spin(dir, 25, pct);
  LDrive_Group.spin(dir, 25, pct);
  wait(150, msec);
  
  // RDrive_Group.spin(dir, 15, pct);
  // LDrive_Group.spin(dir, 15, pct);
  // wait(150, msec);
  
  RDrive_Group.stop();
  LDrive_Group.stop();
}

void testAuton() {
  LDrive_Group.resetPosition();
  RDrive_Group.resetPosition();
  // turn(90);
  while (true) {
    // std::cout << getCurDistance(RDrive_Group) << std::endl;
    turnTo(90, TURN_SPEED, -0.33, 1);
    // driveDistanceNoPID(-1);
    wait(20, msec);
  }
}

void skillsAuton() {
  LDrive_Group.resetPosition();
  RDrive_Group.resetPosition();
  // driveDistance(-50, 70);
  // driveDistanceNoPID(-48, 100);
  // ballsToTheWalls(1400, reverse);
  // gyroscope.calibrate();
  // while (gyroscope.isCalibrating());
  // gyroscope.resetAngle();
  // turnTo(720);

  // std::cout << "Done" <<std::endl;

  // piston.set(false);

  // driveDistance(-36);

  //Push goal to other side of field
  driveDistance(-109);
  wait(250, msec);
  driveDistance(12);
  wait(250, msec);

  //Pick up goal on platform
  // turnTo(116.5);

  turnTo(-64);
  wait(250, msec);
  DriverControl::fourBarPress();
  driveDistance(-26);
  DriverControl::pistonPress();
  DriverControl::fourBarPress();
  wait(250, msec);
  driveDistance(9);
  wait(250, msec);
  //*****Using two bar*****
  // turnTo(100);
  // wait(250, msec);
  // DriverControl::twoBarPress();
  // wait(1000, msec);
  // driveDistance(26);
  // DriverControl::twoBarPress();
  // wait(250, msec);
  // driveDistance(-9);
  // wait(250, msec);
  //**********
  //Leave out this block?
  // turnTo(35);
  // wait(250, msec);
  // driveDistance(16);
  // wait(250, msec);
  // turnTo(-45);
  // wait(250, msec);
  // driveDistance(-22);
  // DriverControl::pistonPress();
  // DriverControl::fourBarPress();
  // wait(250, msec);
  // driveDistance(8);

  //Drive to other side of field
  
  // turnTo(180);
  turnTo(-15);
  gyroscope.resetAngle();
  wait(150, msec);

  // DriverControl::twoBarPress();
  driveDistance(80);
  wait(250, msec);
  driveDistance(-12);
  // wait(250, msec);
  // turnTo(-90);
  // DriverControl::twoBarPress();
  // wait(150, msec);
  // driveDistance(8);
  // wait(250, msec);
  // driveDistance(-8);
  // DriverControl::twoBarReset();

  //Grab the goal in the center of the field
  // turnTo(-80);
  // wait(250, msec);
  // driveDistance(-30);

  return;

  //Push goal to other side of field
  // DriverControl::twoBarPress();
  // wait(1000, msec);
  // driveDistance(104);
  // wait(500, msec);
  // // turnTo(0);
  // driveDistance(-8);
  // wait(250, msec);

  // //Drive to grab goal on platform
  // turnTo(-42);
  // wait(500, msec);
  // driveDistance(25.5);
  // wait(250, msec);
  // DriverControl::twoBarPress();
  // wait(250, msec);
  // driveDistance(-5);

  // //Go back to original side of field
  // turnTo(1);
  // driveDistance(-92);

  // //Pull the goal all the way into our zone
  // driveDistance(4);
  // turnTo(130);
  // DriverControl::twoBarPress();
  // wait(250, msec);
  // driveDistance(-96);
}

void matchAuton() {
  LDrive_Group.resetPosition();
  RDrive_Group.resetPosition();
  // gyroscope.calibrate();
  // while (gyroscope.isCalibrating());

  //Go for yellow goal in front of us
  // DriverControl::fourBarPress();
  // driveDistance(-48, 90);
  // ballsToTheWalls(1600, reverse);
  // wait(250, msec);
  // DriverControl::pistonPress();
  // DriverControl::fourBarPress();
  // wait(250, msec);

  /*****Add slow down later*****/
  // driveDistanceNoPID(36, 50);
  driveDistanceNoPID(39, 50);
  DriverControl::pistonPress();
  wait(500, msec);

  // //Drop off yellow goal
  // driveDistance(32);
  // // driveDistance(12);
  // // turnTo(0);
  // // driveDistance(20);
  // wait(250, msec);
  // // turnTo(-100); //Stupid ass gyro drift
  // turnTo(180);
  // wait(250, msec);
  // DriverControl::fourBarPress();
  // // driveDistance(-12);
  // // DriverControl::pistonPress();
  // // wait(250, msec);

  // driveDistanceNoPID(-12);
  driveDistanceNoPID(-15);
  // turnTo(180);
  turnTo(90, TURN_SPEED, 0, 1);
  turnTo(180, TURN_SPEED, 1, 0);
  DriverControl::pistonPress();
  
  // // driveDistance(12);

  // //Grab win point goal
  // DriverControl::twoBarPress();
  // wait(1000, msec);
  // driveDistance(24);
  // DriverControl::twoBarPress();
  // wait(500, msec);
  // driveDistance(-12);
  // turnTo(-10); //Still hate this gyro drift
  // driveDistance(6);
  // conveyor.spin(reverse, 50, pct);
  // wait(4000, msec);
  // conveyor.stop();
  // DriverControl::twoBarPress();
  // wait(150, msec);
  // driveDistance(-12);
  // DriverControl::twoBarReset();
  turnTo(270, TURN_SPEED, -0.33, 1);
  driveDistanceNoPID(-2);
  DriverControl::clawPress();
  wait(500, msec);

  //Drop preloads
  DriverControl::fourBarPress();
  wait(250, msec);
  conveyor.spin(fwd, 100, pct);

  //Pick up rings
  turnTo(360);
  driveDistanceNoPID(-24);
  turnTo(450);
  driveDistanceNoPID(20);

  return;
  
  //Put goal in corner
  turnTo(340);
  driveDistanceNoPID(-12);

  wait(2, sec);
  conveyor.stop();
  DriverControl::fourBarPress();
  DriverControl::clawPress();

  //Night before WV
  //Go for blue goal on the tape
  // DriverControl::twoBarPress();
  // driveDistance(23);
  // wait(250, msec);
  // turnTo(-92);
  // wait(250, msec);
  // driveDistance(14);
  // DriverControl::twoBarPress();
  // wait(250, msec);
  // driveDistance(-16);
}
