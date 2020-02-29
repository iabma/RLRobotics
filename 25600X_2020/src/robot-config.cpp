#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT2, ratio18_1, false);
motor_group DrivetrainLeft = motor_group(leftMotorA, leftMotorB);

motor rightMotorA = motor(PORT11, ratio18_1, true);
motor rightMotorB = motor(PORT12, ratio18_1, true);
motor_group DrivetrainRight = motor_group(rightMotorA, rightMotorB);

motor intakeRight = motor(PORT17, ratio6_1, true);
motor intakeLeft = motor(PORT9, ratio6_1, false);
motor_group Intake = motor_group(intakeLeft, intakeRight);

motor LadderLift = motor(PORT16, ratio36_1, false);
motor ArmLift = motor(PORT19, ratio36_1, false);

inertial Sensor = inertial(PORT8);
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
bool placing = false;

bool intakeSlow = false;

int ladderLockPos = ladderSittingPos;
int armLockPos = armSittingPos;
int intakeLockPos = 0;
int lockedHeading = 0;

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

const int ladderSittingPos = 0;
const int armSittingPos = 0;
const int ladderExtended = 1450;
const int ladderMax = ladderExtended + 100;

const int ladderNod = 750;
const int midTower = 1200;
const int lowTower = 940;

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
      DrivetrainLeft.stop();
      DrivetrainLNeedsToBeStopped = false;
    }
  } else {
    DrivetrainLNeedsToBeStopped = true;
  }
  if (drivetrainRightSideSpeed < deadband &&
      drivetrainRightSideSpeed > -deadband) {
    if (DrivetrainRNeedsToBeStopped) {
      DrivetrainRight.stop();
      DrivetrainRNeedsToBeStopped = false;
    }
  } else {
    DrivetrainRNeedsToBeStopped = true;
  }
}

int clamp(int val, int min, int max) {
  return (val < min) ? min : (max < val) ? max : val;
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

  Intake.setVelocity(intakeSlow ? 50 : 100, velocityUnits::pct);
  Intake.setStopping(hold);
  LadderLift.setStopping(hold);
  ArmLift.setStopping(hold);
  LadderLift.setVelocity(100, pct);
  ArmLift.setVelocity(100, pct);
  ArmLift.rotateTo(armSittingPos, deg);
  LadderLift.rotateTo(ladderSittingPos, deg);
  
  while (true) {
    printf("%f\n", Sensor.heading());
    if (RemoteControlCodeEnabled) {
      /* DRIVETRAIN */
      int drivetrainLeftSideSpeed =
          pow(Controller.Axis3.position() + Controller.Axis1.position(), 3) / 10000; // cubic sensitivity 
      int drivetrainRightSideSpeed =
          pow(Controller.Axis3.position() - Controller.Axis1.position(), 3) / 10000;

      drivetrainLeftSideSpeed = Controller.ButtonL1.pressing() ? drivetrainLeftSideSpeed / 4 : drivetrainLeftSideSpeed;
      drivetrainRightSideSpeed = Controller.ButtonL1.pressing() ? drivetrainRightSideSpeed / 4 : drivetrainRightSideSpeed;
      
      checkDeadbands(drivetrainLeftSideSpeed, drivetrainRightSideSpeed);

      if (DrivetrainLNeedsToBeStopped) {
        DrivetrainLeft.setVelocity(drivetrainLeftSideSpeed, percent);
        DrivetrainLeft.spin(forward);
      }

      if (DrivetrainRNeedsToBeStopped) {
        DrivetrainRight.setVelocity(drivetrainRightSideSpeed, percent);
        DrivetrainRight.spin(forward);
      }

      /* ARM CODE */
      int armPos = ArmLift.rotation(deg);

      if (Controller.ButtonDown.pressing() && armPos > 0) {
        ArmLift.spin(reverse);
        ArmLiftStopped = false;
      } else if (Controller.ButtonRight.pressing()) { // ADD UPPER LIMIT
        ArmLift.spin(forward);
        ArmLiftStopped = false;
      } else if (!ArmLiftStopped) {
        ArmLift.stop();
        ArmLiftStopped = true;
        //armLockPos = clamp(armPos, armSittingPos / 2, midTower);
      } /* else if (ArmLiftStopped && armPos != armLockPos) {
        ArmLift.rotateTo(armLockPos, deg, 100, velocityUnits::pct, false);
      } */
      
      /* if (ladderLockPos <= ladderNod && armLockPos > armSittingPos)
        ladderLockPos = clamp(ladderSittingPos + (armPos - armSittingPos) * 10, ladderSittingPos, ladderNod); */

      /* LADDER LIFTER */
      int ladderPos = LadderLift.rotation(deg);
      int ladderSpeed = clamp(pow(double(ladderExtended - ladderPos) / double(ladderExtended), .5) * 100 + 20, 50, 100);
      
      if (Controller.ButtonX.pressing() && ladderPos <= ladderMax) {
        //printf("ladderpos: %d\n", ladderPos);
        placing = true;
        Intake.setStopping(coast);
        LadderLift.spin(forward, ladderSpeed, pct);
        LadderLiftStopped = false;
      } else if (Controller.ButtonB.pressing() && ladderPos >= ladderSittingPos) {
        placing = true;
        Intake.setStopping(coast);
        LadderLift.spin(reverse, 100, pct);
        LadderLiftStopped = false;
      } else if (!LadderLiftStopped) {
        placing = false;
        LadderLift.stop();
        LadderLiftStopped = true;
        //Intake.setStopping(hold);
        //ladderLockPos = clamp(ladderPos, ladderSittingPos, ladderExtended);
      } /* else if (LadderLiftStopped && ladderPos != ladderLockPos) {
        LadderLift.rotateTo(ladderLockPos, deg, 100, velocityUnits::pct, false);
      } */

      /* INTAKE */
      if (Controller.ButtonR1.pressing()) {
        Intake.setStopping(hold);
        Intake.spin(forward, 100, pct);
        IntakeStopped = false;
      } else if (Controller.ButtonR2.pressing()) {
        Intake.setStopping(hold);
        Intake.spin(reverse, 75, pct);
        IntakeStopped = false;
      } else if (!IntakeStopped && !placing) {
        Intake.stop();
        //intakeLockPos = intakePos;
        IntakeStopped = true;
      } /* else if (IntakeStopped && intakePos != intakeLockPos) {
        Intake.rotateTo(intakeLockPos, deg, 100, velocityUnits::pct, false);
      } */

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
        armData[0] = clamp(unclamped, 0, 53);

        /* ladder */
        unclamped = LadderLift.rotation(deg) / 9 + 2;
        ladderData[0] = clamp(unclamped, 0, 53);

        /* drivetrain */
        unclamped = drivetrainLeftSideSpeed / 4 + 24;
        drivetrainLeftData[0] = clamp(unclamped, 0, 49);
        unclamped = drivetrainRightSideSpeed / 4 + 25;
        drivetrainRightData[0] = clamp(unclamped, 0, 49);

        /* intake */
        unclamped = Intake.velocity(pct) / 2 + 24;
        intakeData[0] = clamp(unclamped, 0, 49);

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