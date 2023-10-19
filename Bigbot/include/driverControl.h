#ifndef DRIVER_CONTROL_H
#define DRIVER_CONTROL_H

#include "vex.h"

#define CONVEYOR_SPEED 100

using namespace vex;

class DriverControl {
public:
  //Two bar
  static bool clawToggle;
  static void clawPress();
  static void clawUp();
  static void clawDown();
  static void clawReset();

  //Four bar
  static bool fourBarToggle;
  static void fourBarPress();

  //Pneumatics
  static bool pistonToggle;
  static void pistonPress();

public:
  // TestControl();
  void Run();
};

#endif
