#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT2, ratio18_1, false);
motor leftMotorB = motor(PORT10, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);

motor rightMotorA = motor(PORT3, ratio18_1, true);
motor rightMotorB = motor(PORT1, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);

drivetrain Drivetrain =
    drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 385, 240, mm, 1);
motor LadderLift = motor(PORT11, ratio36_1, false);
motor ArmLift = motor(PORT5, ratio36_1, false);
controller Controller1 = controller(primary);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1UpDownButtonsControlMotorsStopped = true;
bool Controller1XBButtonsControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

int deadband = 8;

// Check if joystick is within deadband ranges
void checkDeadbands(int drivetrainLeftSideSpeed, int drivetrainRightSideSpeed) {
  if (drivetrainLeftSideSpeed < deadband &&
      drivetrainLeftSideSpeed > -deadband) {
    // check if the left motor has already been stopped
    if (DrivetrainLNeedsToBeStopped_Controller1) {
      // stop the left drive motor
      LeftDriveSmart.stop();
      // tell the code that the left motor has been stopped
      DrivetrainLNeedsToBeStopped_Controller1 = false;
    }
  } else {
    // reset the toggle so that the deadband code knows to stop the left motor
    // next time the input is in the deadband range
    DrivetrainLNeedsToBeStopped_Controller1 = true;
  }
  if (drivetrainRightSideSpeed < deadband &&
      drivetrainRightSideSpeed > -deadband) {
    // check if the right motor has already been stopped
    if (DrivetrainRNeedsToBeStopped_Controller1) {
      // stop the right drive motor
      RightDriveSmart.stop();
      // tell the code that the right motor has been stopped
      DrivetrainRNeedsToBeStopped_Controller1 = false;
    }
  } else {
    // reset the toggle so that the deadband code knows to stop the right motor
    // next time the input is in the deadband range
    DrivetrainRNeedsToBeStopped_Controller1 = true;
  }
}

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_callback_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while (true) {
    if (RemoteControlCodeEnabled) {
      /* DRIVETRAIN */
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();

      checkDeadbands(drivetrainLeftSideSpeed, drivetrainRightSideSpeed);

      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }

      /* ARM CODE */

      if (Controller1.ButtonUp.pressing()) {
        ArmLift.spin(forward);
        Controller1UpDownButtonsControlMotorsStopped = false;
      } else if (Controller1.ButtonDown.pressing()) {
        ArmLift.spin(reverse);
        Controller1UpDownButtonsControlMotorsStopped = false;
      } else if (!Controller1UpDownButtonsControlMotorsStopped) {
        ArmLift.stop();
        Controller1UpDownButtonsControlMotorsStopped = true;
      }

      /* LADDER LIFTER */
      if (Controller1.ButtonX.pressing()) {
        LadderLift.spin(forward);
        Controller1XBButtonsControlMotorsStopped = false;
      } else if (Controller1.ButtonB.pressing()) {
        LadderLift.spin(reverse);
        Controller1XBButtonsControlMotorsStopped = false;
      } else if (!Controller1XBButtonsControlMotorsStopped) {
        LadderLift.stop();
        Controller1XBButtonsControlMotorsStopped = true;
      }
    }

    wait(10, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_callback_Controller1);
}