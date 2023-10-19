#ifndef AUTON_H
#define AUTON_H

#include "vex.h"
#include "Pid.h"
#include "driverControl.h"

using namespace vex;

#define GEAR_RATIO 5/3

void driveDistance(double distance, int speed);
void driveDistanceNoPID(double distance, int speed);
void ballsToTheWalls(double time);
void turn(double theta, int speed);
void turnTo(double theta, int speed);
void testAuton();
void skillsAuton();
void matchAuton();
extern Pid turnPID, rDrivePID, lDrivePID;

#endif
