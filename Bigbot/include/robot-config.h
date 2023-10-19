using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor RMotor1;
extern motor RMotor2;
extern motor RMotor3;
extern motor RMotor4;
extern motor RMotor5;
extern motor LMotor1;
extern motor LMotor2;
extern motor LMotor3;
extern motor LMotor4;
extern motor LMotor5;
extern motor_group claw;
extern digital_out piston;
extern gyro gyroscope;
extern motor_group fourBar;
extern motor conveyor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );