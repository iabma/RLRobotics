/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Ian Balaguera                                             */
/*    Created:      Thu Jan 02 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

competition comp = competition();

/* CONSTANTS */

void flipOut() {
  if (comp.isCompetitionSwitch() || comp.isFieldControl()) {
    ArmLift.setVelocity(100, pct);
    LadderLift.setVelocity(100, pct);
    LadderLift.rotateTo(ladderExtended * .55, deg, false);
    Intake.spin(reverse, 100, pct);
    ArmLift.rotateFor(fwd, 700, deg, true);
    Intake.stop();
    ArmLift.rotateFor(fwd, 400, deg, true);
    wait(500, msec);
    ArmLift.rotateTo(lowTower * .9, deg, true);
  }
}

void drive(vex::directionType dir, int rotation, int speed, bool waitForCompletion) {
  DrivetrainLeft.setVelocity(speed, pct);
  DrivetrainRight.setVelocity(speed, pct);
  DrivetrainLeft.spinFor(dir, rotation, deg, false);
  DrivetrainRight.spinFor(dir, rotation, deg, waitForCompletion);
}

void rotateTo(int targetHeading, int speed) {
  DrivetrainLeft.setVelocity(speed, pct);
  DrivetrainRight.setVelocity(speed, pct);
  int currentHeading = Sensor.heading();
  while (abs(currentHeading - targetHeading) > 1) {
    //printf("currentHeading: %d | targetHeading: %d\n", currentHeading, targetHeading);
    int leftTurnDelta = currentHeading - targetHeading;
    int rightTurnDelta = targetHeading - currentHeading;
    int leftTurnAngle = leftTurnDelta < 0 ? leftTurnDelta + 360 : leftTurnDelta;
    int rightTurnAngle = rightTurnDelta < 0 ? rightTurnDelta + 360 : rightTurnDelta;
    if (leftTurnAngle < rightTurnAngle) {
      DrivetrainLeft.spin(reverse);
      DrivetrainRight.spin(forward);
    } else {
      DrivetrainLeft.spin(forward);
      DrivetrainRight.spin(reverse);
    }
    wait(5, msec);
    DrivetrainLeft.stop();
    DrivetrainRight.stop();
    currentHeading = Sensor.heading();
  }
  DrivetrainLeft.stop();
  DrivetrainRight.stop();
}

void accelerate(int rotation, int finalSpeed) {
  int trainPos = DrivetrainLeft.rotation(deg);
  int trainStartPos = trainPos;
  int speed = 10;
  int holdHeading = Sensor.heading();
  while (trainPos < trainStartPos + rotation) {
    //printf("trainPos: %d\n", trainPos);
    speed = speed < finalSpeed ? speed + 1 : speed;
    DrivetrainLeft.spin(fwd, speed, pct);
    DrivetrainRight.spin(fwd, speed, pct);
    wait(40, msec);
    //rotateTo(holdHeading, 50);
    trainPos = DrivetrainLeft.rotation(deg);
  }
}

int calculateLadderSpeed(int ladderPos) {
  double pos = double(ladderPos) / double(ladderExtended) * 10;
  //printf("%f\n", pos);
  //return 80 * pow(10.1 - pos, 0.2);
  if (pos < 6)
    return 100;
  else if (pos < 7.5)
    return 80 * pow(10.1 - pos, 0.2) - 6;
  else if (pos < 7.7)
    return 30;
  else
    return 20 * tanh(9 - pos) + 50;
}

void placeStack() {
  int ladderPos = LadderLift.rotation(deg);
  int ladderSpeed = calculateLadderSpeed(ladderPos);
  placing = true;
  IntakeStopped = false;
  LadderLiftStopped = false;
  Intake.setStopping(coast);

  while (ladderPos < ladderExtended) {
    //Intake.spin(reverse, ladderSpeed / 10, pct);
    LadderLift.spin(forward, ladderSpeed, pct);
    ladderPos = LadderLift.rotation(deg);
    ladderSpeed = calculateLadderSpeed(ladderPos);
    if (ladderPos < ladderExtended * .4)
      Intake.spin(forward, 40, pct);
    else if (ladderPos > ladderExtended * .5)
      Intake.spin(reverse, 20, pct);
    wait(10, msec);
  }

  Intake.stop();

  placing = false;
  Intake.setStopping(hold);
  //Intake.stop();
  IntakeStopped = true;
  LadderLift.stop();
  LadderLiftStopped = true;
}

void unprotectedAuton(int auton, int turning) {
  rotateTo(0, 50);
  IntakeStopped = true;
  Intake.spin(fwd, 100, pct);
  accelerate(1000, 30);
  rotateTo(auton == 4 ? 360 - turning : turning, 40);
  drive(fwd, 600, 90, true);
  Intake.stop();
  //Intake.spin(reverse, 100, pct);
  drive(fwd, 500, 60, false);
  wait(100, msec);
  placeStack();
  drive(reverse, 100, 100, true);
}

void protectedAuton(int auton, int turning) {
  IntakeStopped = true;
  Intake.spin(fwd, 100, pct);
  accelerate(600, 35);
  rotateTo(auton == 1 ? 360 - turning : turning, 30);
  Intake.stop();
  drive(fwd, 600, 60, true);
  //rotateTo(auton == 1 ? 205 : 115, 40);
  //drive(fwd, 300, 60, true);
  placeStack();
  drive(reverse, 400, 70, true);
}

void autonomous() {
  flipOut();
  ArmLift.rotateTo(armSittingPos, deg, false);
  LadderLift.rotateTo(ladderSittingPos, deg, true); // PLS FIX MAYBE?

  int unprotectedTurning = 125;
  int protectedTurning = 120;

  switch (selectedAuton) {
    case 0:
      //accelerate(650, 50);
      //drive(reverse, 300, 70, true);
      break;
    case 1: // redpro
      protectedAuton(selectedAuton, protectedTurning);
      break;
    case 2: // redun
      unprotectedAuton(selectedAuton, unprotectedTurning);
      break;
    case 3: // bluepro
      protectedAuton(selectedAuton, protectedTurning);
      break;
    case 4: // blueun
      unprotectedAuton(selectedAuton, unprotectedTurning);
      break;
  }
}

void drivercontrol() {
  /* ARM */
  Controller.ButtonUp.pressed([]() {
    LadderLift.setVelocity(100, pct);
    ArmLift.setVelocity(100, pct);
    LadderLift.rotateTo(ladderNod * .8, deg, false);
    ArmLift.rotateTo(midTower, deg, true);
    LadderLift.stop();
    ArmLift.stop();
  });

  Controller.ButtonLeft.pressed([]() {
    LadderLift.setVelocity(100, pct);
    ArmLift.setVelocity(100, pct);
    LadderLift.rotateTo(ladderNod * .9, deg, false);
    ArmLift.rotateTo(lowTower, deg, true);
    LadderLift.stop();
    ArmLift.stop();
  });

  /* LADDER */
  Controller.ButtonY.pressed([]() {
    LadderLift.setVelocity(100, pct);
    ArmLift.setVelocity(100, pct);
    ArmLift.rotateTo(armSittingPos, deg, false);
    int armPos = ArmLift.rotation(deg);
    while (armPos > lowTower * .9) {
      wait(10, msec);
      armPos = ArmLift.rotation(deg);
    }
    /* placing = true;
    IntakeStopped = false; */
    Intake.setStopping(coast);
    LadderLift.rotateTo(ladderSittingPos, deg, true);
    Intake.setStopping(hold);
    /* placing = false;
    IntakeStopped = true; */
  });

  Controller.ButtonA.pressed([]() {
    LadderLiftStopped = true;
    ArmLiftStopped = true;
    Intake.setVelocity(30, pct);
    Intake.rotateFor(reverse, 250, deg);
    armLockPos = armSittingPos;
    ladderLockPos = ladderExtended;
  });

  /* Intake */
  Controller.ButtonL2.pressed([]() { 
    placeStack();
    //intakeSlow = !intakeSlow; 
  });
}

int main() {
  vexcodeInit();
  Sensor.calibrate();

  comp.autonomous(autonomous);
  comp.drivercontrol(drivercontrol);
}
