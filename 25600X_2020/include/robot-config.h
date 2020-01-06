using namespace vex;

extern brain Brain;

/* GLOBAL */
extern digital_out PneumaticLeft;
extern digital_out PneumaticRight;
extern drivetrain Drivetrain;
extern motor LadderLift;
extern motor ArmLift;
extern controller Controller;
extern motor_group Intake;

extern int selectedAuton;
extern bool LadderLiftStopped;
extern bool ArmLiftStopped;
extern int ladderLockPos;
extern int armLockPos;
extern const int ladderExtended;
extern const int ladderSittingPos;
extern const int armSittingPos;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );