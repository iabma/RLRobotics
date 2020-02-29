using namespace vex;

extern brain Brain;

/* GLOBAL */
extern motor_group DrivetrainRight;
extern motor_group DrivetrainLeft;
extern motor_group Intake;
extern motor LadderLift;
extern motor ArmLift;
extern controller Controller;
extern inertial Sensor;

extern int selectedAuton;
extern bool LadderLiftStopped;
extern bool ArmLiftStopped;
extern bool IntakeStopped;
extern bool intakeSlow;
extern bool placing;
extern int ladderLockPos;
extern int armLockPos;
extern const int ladderExtended;
extern const int ladderSittingPos;
extern const int armSittingPos;
extern const int ladderNod;
extern const int midTower;
extern const int lowTower;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
int clamp(int val, int min, int max);