#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

motor leftMotorA = motor(PORT14, ratio18_1, false);
motor leftMotorB = motor(PORT11, ratio18_1, false);
motor_group DrivetrainLeft = motor_group(leftMotorA, leftMotorB);

motor rightMotorA = motor(PORT7, ratio18_1, true);
motor rightMotorB = motor(PORT1, ratio18_1, true);
motor_group DrivetrainRight = motor_group(rightMotorA, rightMotorB);

motor intakeRight = motor(PORT20, ratio6_1, true);
motor intakeLeft = motor(PORT10, ratio6_1, false);
motor_group Intake = motor_group(intakeLeft, intakeRight);

motor Ladder = motor(PORT16, ratio36_1, false);
motor Arm = motor(PORT19, ratio36_1, false);

inertial Sensor = inertial(PORT3);
controller Controller = controller(primary);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
bool IntakeStopped = true;

void vexcodeInit(void) {
  Arm.setStopping(hold);
  Intake.setStopping(hold);
  Arm.rotateTo(40, deg);
  while (true) {
    if (Controller.ButtonR1.pressing()) {
      Intake.setStopping(hold);
      Intake.spin(forward, 100, pct);
      IntakeStopped = false;
    } else if (Controller.ButtonR2.pressing()) {
      Intake.setStopping(hold);
      Intake.spin(reverse, 75, pct);
      IntakeStopped = false;
    } else if (!IntakeStopped) {
      Intake.stop();
      // intakeLockPos = intakePos;
      IntakeStopped = true;
    }
  }
  // Nothing to initialize
}