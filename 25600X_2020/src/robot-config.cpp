#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT6, ratio18_1, false);
motor leftMotorB = motor(PORT2, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);

motor rightMotorA = motor(PORT3, ratio18_1, true);
motor rightMotorB = motor(PORT4, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);

motor intakeRight = motor(PORT7, ratio6_1, false); // UPDATE PORTS
motor intakeLeft = motor(PORT17, ratio6_1, true);
motor_group Intake = motor_group(intakeLeft, intakeRight);

digital_out PneumaticLeft = digital_out(Brain.ThreeWirePort.D);
digital_out PneumaticRight = digital_out(Brain.ThreeWirePort.A);

drivetrain Drivetrain =
    drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 385, 240, mm, 1);
motor LadderLift = motor(PORT11, ratio36_1, true);
motor ArmLift = motor(PORT5, ratio36_1, false);
controller Controller = controller(primary);

/* VARIABLES */

/* robot variables */
bool RemoteControlCodeEnabled = true;
int selectedAuton = 0; // 0 - none | 1 - redpro | 2 - redun | 3 - bluepro | 4 - blueun

bool ArmLiftStopped = true;
bool LadderLiftStopped = true;
bool DrivetrainLNeedsToBeStopped = true;
bool DrivetrainRNeedsToBeStopped = true;
bool IntakeStopped = true;

/* ui variables */
int UIClock = 0;
int* armData = new int[206];
int* ladderData = new int[206];
int* drivetrainLeftData = new int[206];
int* drivetrainRightData = new int[206];
int* intakeData = new int[206];

/* CONSTANTS */

/* robot constants */
const int deadband = 5;
const int clockms = 10;

const int ladderSittingPos = 39;
const int armSittingPos = 60;
const int ladderExtended = 485;

int ladderLockPos = ladderSittingPos;
int armLockPos = armSittingPos;

/* ui constants */
const int UIUpdateFrequency = 2;

const int redProtectedX = 399;
const int redProtectedY = 35;
const int redUnprotectedX = 331;
const int redUnprotectedY = 35;
const int blueProtectedX = 399;
const int blueProtectedY = 205;
const int blueUnprotectedX = 331;
const int blueUnprotectedY = 205;

// Check if joystick is within deadband ranges
void checkDeadbands(int drivetrainLeftSideSpeed, int drivetrainRightSideSpeed) {
  if (drivetrainLeftSideSpeed < deadband &&
      drivetrainLeftSideSpeed > -deadband) {
    if (DrivetrainLNeedsToBeStopped) {
      LeftDriveSmart.stop();
      DrivetrainLNeedsToBeStopped = false;
    }
  } else {
    DrivetrainLNeedsToBeStopped = true;
  }
  if (drivetrainRightSideSpeed < deadband &&
      drivetrainRightSideSpeed > -deadband) {
    if (DrivetrainRNeedsToBeStopped) {
      RightDriveSmart.stop();
      DrivetrainRNeedsToBeStopped = false;
    }
  } else {
    DrivetrainRNeedsToBeStopped = true;
  }
}

double distanceBetweenPoints(int x_1, int y_1, int x_2, int y_2) {
  return sqrt((x_1-x_2)*(x_1-x_2)+(y_1-y_2)*(y_1-y_2));
}

void defaultAutons() {
  Brain.Screen.setPenWidth(1);
  Brain.Screen.setPenColor(0xFF929292);
  Brain.Screen.setFillColor(0xFF626262);
  Brain.Screen.drawCircle(redProtectedX, redProtectedY, 12); // red protected
  Brain.Screen.drawCircle(redUnprotectedX, redUnprotectedY, 12); // red unprotected
  Brain.Screen.drawCircle(blueProtectedX, blueProtectedY, 12); // blue protected
  Brain.Screen.drawCircle(blueUnprotectedX, blueUnprotectedY, 12); // blue unprotected
}

void updateAutonSelection(int selected, int x, int y) {
  selectedAuton = selected;
  defaultAutons();
  Brain.Screen.setPenWidth(1);
  Brain.Screen.setPenColor(0xFF00D615);
  Brain.Screen.setFillColor(0xFF00AD1E);
  Brain.Screen.drawCircle(x, y, 12);
}

void UISetup() {
  /* 
    NOTES
    screen dimensions - 480x240
    hex values - argb
  */

  /* BACKGROUND */

  Brain.Screen.setFillColor(0xFF212121);
  Brain.Screen.setPenColor(0xFF212121);
  Brain.Screen.drawRectangle(0, 0, 480, 240);
  /* field */
  Brain.Screen.setFillColor(0xFF424242);
  Brain.Screen.setPenColor(0xFFD6D6D6);
  Brain.Screen.setPenWidth(2);
  Brain.Screen.drawRectangle(256, 17, 206, 206);
  Brain.Screen.setPenWidth(0);
  /* red */
  Brain.Screen.setFillColor(0xFFD21F48);
  Brain.Screen.drawRectangle(394, 18, 34, 34); // outer square
  Brain.Screen.drawRectangle(428, 52, 34, 34); // inner square
  Brain.Screen.drawRectangle(443, 42, 19, 3); // protected zone 
  Brain.Screen.drawRectangle(442, 18, 3, 26); //
  Brain.Screen.drawRectangle(274, 18, 3, 19); // unprotected zone
  Brain.Screen.drawRectangle(257, 35, 19, 3); //
  /* blue */
  Brain.Screen.setFillColor(0xFF3798EE);
  Brain.Screen.drawRectangle(394, 189, 34, 34); // same order as red
  Brain.Screen.drawRectangle(428, 155, 34, 34);
  Brain.Screen.drawRectangle(443, 196, 19, 3);
  Brain.Screen.drawRectangle(442, 197, 3, 26);
  Brain.Screen.drawRectangle(274, 204, 3, 19);
  Brain.Screen.drawRectangle(257, 203, 19, 3);
  /* tape */
  Brain.Screen.setPenColor(0xFFEBEBEB);
  Brain.Screen.setPenWidth(1);
  // red tape
  Brain.Screen.drawLine(428, 18, 428, 51); // left vertical
  Brain.Screen.drawLine(393, 18, 393, 51); // right vertical
  Brain.Screen.drawLine(394, 52, 427, 85); // diagonal
  Brain.Screen.drawLine(461, 51, 428, 51); // top horizontal
  Brain.Screen.drawLine(461, 85, 428, 85); // bottom horizontal
  // blue tape
  Brain.Screen.drawLine(428, 190, 428, 222); // same order as red
  Brain.Screen.drawLine(393, 190, 393, 222);
  Brain.Screen.drawLine(394, 189, 427, 156);
  Brain.Screen.drawLine(461, 155, 428, 155);
  Brain.Screen.drawLine(461, 189, 428, 189);
  // middle tape
  Brain.Screen.drawLine(460, 118, 258, 118);
  Brain.Screen.drawLine(460, 121, 258, 121);

  /* AUTON SPOTS */

  defaultAutons();

  /* GRAPH LABELS */
  Brain.Screen.setFillColor(0xFF212121);
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.printAt(197, 14, "Arm");
  Brain.Screen.printAt(131, 14, "Ladd.");
  Brain.Screen.printAt(80, 14, "Trn.");
  Brain.Screen.printAt(23, 14, "Intk");
}

void activeUI() {
  /* CHECK BUTTON PRESSES */

  Brain.Screen.pressed([](){
    int x = Brain.Screen.xPosition();
    int y = Brain.Screen.yPosition();

    if (distanceBetweenPoints(x, y, redProtectedX, redProtectedY) < 50.0) {
      if (selectedAuton == 1) {
        selectedAuton = 0;
        defaultAutons();
      } else
        updateAutonSelection(1, redProtectedX, redProtectedY);
    } else if (distanceBetweenPoints(x, y, redUnprotectedX, redUnprotectedY) < 50.0) {
      if (selectedAuton == 2) {
        selectedAuton = 0;
        defaultAutons();
      } else
        updateAutonSelection(2, redUnprotectedX, redUnprotectedY);
    } else if (distanceBetweenPoints(x, y, blueProtectedX, blueProtectedY) < 50.0) {
      if (selectedAuton == 3) {
        selectedAuton = 0;
        defaultAutons();
      } else
        updateAutonSelection(3, blueProtectedX, blueProtectedY);
    } else if (distanceBetweenPoints(x, y, blueUnprotectedX, blueUnprotectedY) < 50.0) {
      if (selectedAuton == 4) {
        selectedAuton = 0;
        defaultAutons();
      } else
        updateAutonSelection(4, blueUnprotectedX, blueUnprotectedY);
    }
  });
}

int setup() {
  UISetup();
  activeUI();

  Intake.setVelocity(100, velocityUnits::pct);
  
  while (true) {
    if (RemoteControlCodeEnabled) {
      /* DRIVETRAIN */
      int drivetrainLeftSideSpeed =
          Controller.Axis3.position() + Controller.Axis1.position();
      int drivetrainRightSideSpeed =
          Controller.Axis3.position() - Controller.Axis1.position();

      /*
      int drivetrainLeftSideSpeed =
          cos(1.57 * Controller.Axis3.position() / 100.0) * 100 + cos(1.57 * Controller.Axis1.position() / 100.0) * 100;
      int drivetrainRightSideSpeed =
          cos(1.57 * Controller.Axis3.position() / 100.0) * 100 - cos(1.57 * Controller.Axis1.position() / 100.0) * 100;
      */

      drivetrainLeftSideSpeed = Controller.ButtonL1.pressing() ? drivetrainLeftSideSpeed / 5 : drivetrainLeftSideSpeed / 2;
      drivetrainRightSideSpeed = Controller.ButtonL1.pressing() ? drivetrainRightSideSpeed / 5 : drivetrainRightSideSpeed / 2;

      checkDeadbands(drivetrainLeftSideSpeed, drivetrainRightSideSpeed);

      if (DrivetrainLNeedsToBeStopped) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }

      if (DrivetrainRNeedsToBeStopped) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }

      /* ARM CODE */
      int armPos = ArmLift.rotation(deg);
      //Brain.Screen.printAt(5, 20, "Armpos: %d", armPos);

      if (Controller.ButtonDown.pressing()) {
        ArmLift.spin(reverse);
        ArmLiftStopped = false;
      } else if (Controller.ButtonRight.pressing()) {
        ArmLift.spin(forward);
        ArmLiftStopped = false;
      } else if (!ArmLiftStopped) {
        ArmLift.stop();
        ArmLiftStopped = true;
        armLockPos = armSittingPos > armPos ? armSittingPos : armPos;
      } else if (ArmLiftStopped && armPos != armLockPos) {
        ArmLift.rotateTo(armLockPos, deg, 100, velocityUnits::pct, false);
      }

      /* LADDER LIFTER */
      int ladderPos = LadderLift.rotation(deg);
      //Brain.Screen.printAt(5, 45, "Ladderpos: %d", ladderPos);

      if (Controller.ButtonX.pressing()) {
        if (ladderPos <= 485) {
          LadderLift.spin(forward);
          LadderLiftStopped = false;
        } else {
          LadderLift.stop();
          LadderLiftStopped = true;
          ladderLockPos = ladderSittingPos > ladderPos ? ladderSittingPos : ladderPos;
        }
      } else if (Controller.ButtonB.pressing()) {
        LadderLift.spin(reverse);
        LadderLiftStopped = false;
      } else if (!LadderLiftStopped) {
        LadderLift.stop();
        LadderLiftStopped = true;
        ladderLockPos = ladderSittingPos > ladderPos ? ladderSittingPos : ladderPos;
      } else if (LadderLiftStopped && ladderPos != ladderLockPos) {
        LadderLift.rotateTo(ladderLockPos, deg, 100, velocityUnits::pct, false);
      }

      /* INTAKE */
      if (Controller.ButtonR1.pressing()) {
        Intake.spin(forward);
        IntakeStopped = false;
      } else if (Controller.ButtonR2.pressing()) {
        Intake.spin(reverse);
        IntakeStopped = false;
      } else if (!IntakeStopped) {
        Intake.stop();
        IntakeStopped = true;
      }

      /* GRAPHED DATA */

      UIClock++;
      if (UIClock == UIUpdateFrequency) {
        UIClock = 0;

        for (int i = 205; i >= 0; i--) {
          armData[i] = armData[i - 1];
          ladderData[i] = ladderData[i - 1];
          drivetrainLeftData[i] = drivetrainLeftData[i - 1];
          drivetrainRightData[i] = drivetrainRightData[i - 1];
          intakeData[i] = intakeData[i - 1];
        }

        /* backgrounds */
        Brain.Screen.setPenColor(0xFF353535);
        Brain.Screen.setFillColor(0xFF2F2F2F);
        Brain.Screen.drawRectangle(185, 17, 54, 206);
        Brain.Screen.drawRectangle(127, 17, 54, 206);
        Brain.Screen.drawRectangle(73, 17, 50, 206);
        Brain.Screen.drawRectangle(17, 17, 50, 206);

        /* arm */
        int unclamped = ArmLift.rotation(deg) / 10 + 1;
        armData[0] = (unclamped < 0) ? 0 : (53 < unclamped) ? 53 : unclamped;

        /* ladder */
        unclamped = LadderLift.rotation(deg) / 9 + 2;
        ladderData[0] = (unclamped < 0) ? 0 : (53 < unclamped) ? 53 : unclamped;

        /* drivetrain */
        unclamped = drivetrainLeftSideSpeed / 4 + 24;
        drivetrainLeftData[0] = (unclamped < 0) ? 0 : (49 < unclamped) ? 49 : unclamped;
        unclamped = drivetrainRightSideSpeed / 4 + 25;
        drivetrainRightData[0] = (unclamped < 0) ? 0 : (49 < unclamped) ? 49 : unclamped;

        /* intake */
        unclamped = Intake.velocity(pct) / 2 + 24;
        intakeData[0] = (unclamped < 0) ? 0 : (49 < unclamped) ? 49 : unclamped;

        /* drawing */
        for (int i = 0; i < 206; i++) {
          Brain.Screen.setPenColor(0xFFFF8F16);
          Brain.Screen.drawPixel(185 + armData[i], 222 - i);
          Brain.Screen.setPenColor(0xFF43D6FF);
          Brain.Screen.drawPixel(127 + ladderData[i], 222 - i);
          Brain.Screen.setPenColor(0xFF8BFF5E);
          Brain.Screen.drawPixel(73 + drivetrainRightData[i], 222 - i);
          Brain.Screen.setPenColor(0xFFF76DFF);
          Brain.Screen.drawPixel(73 + drivetrainLeftData[i], 222 - i);
          Brain.Screen.setPenColor(0xFFE7FF3E);
          Brain.Screen.drawPixel(17 + intakeData[i], 222 - i);
        }
      }
    }

    wait(clockms, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  task rc_auto_loop_task_Controller(setup);
}