#ifndef AUTON_H
#define AUTON_H

#include "vex.h"
#include "Pid.h"
#include "driverControl.h"

using namespace vex;

#define GEAR_RATIO 7/3.0

// class Auton {
//   public:
//     Auton();
    void driveDistance(double distance, int speed);
    void turn(double theta, int speed);
    void turnTo(double theta, int speed);
    void testAuton();
    void skillsAuton();
    void matchAuton();
    void BenMatch();
    // Pid turnPID, rDrivePID, lDrivePID;
// };

#endif
