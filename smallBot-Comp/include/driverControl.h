#include "vex.h"
// #include "Pid.h"
#include <iostream>

#define CONVEYOR_SPEED 87

using namespace vex;

class DriverControl {
public:
  static bool ClawToggle;
  static void ClawPress();
  static bool Claw4Toggle;
  static void Claw4Press();
  static bool ArmToggle;
  static void ArmPress();
  static void armsLight();
  static void armDown();
public:
  // TestControl();
  void Run();
};