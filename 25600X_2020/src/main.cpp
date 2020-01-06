/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       IB8                                                       */
/*    Created:      Thu Jan 02 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    2, 10, 3, 1
// LadderLift           motor         11
// ArmLift              motor         5
// Controller1          controller
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

competition comp = competition();

/* CONSTANTS */

const int ladderNod = 250;
const int midTower = 530;
const int lowTower = 430;

void autonomous() {
  /* Drivetrain.driveFor(20, distanceUnits::in, 100, velocityUnits::pct);
  wait(100, msec); */
}

void drivercontrol() {
  /* ARM */
  Controller.ButtonUp.pressed([]() {
    LadderLiftStopped = true;
    ArmLiftStopped = true;
    ladderLockPos = ladderNod;
    armLockPos = midTower;
    /* LadderLift.rotateTo(ladderNod, deg, 100, velocityUnits::pct, false);
    ArmLift.rotateTo(midTower, deg, 100, velocityUnits::pct, false); */
  });

  Controller.ButtonLeft.pressed([]() {
    LadderLiftStopped = true;
    ArmLiftStopped = true;
    ladderLockPos = ladderNod;
    armLockPos = lowTower;
    /* LadderLift.rotateTo(ladderNod, deg, 100, velocityUnits::pct, false);
    ArmLift.rotateTo(lowTower, deg, 100, velocityUnits::pct, false); */
  });

  /* LADDER */
  Controller.ButtonY.pressed([]() {
    LadderLiftStopped = true;
    ArmLiftStopped = true;
    armLockPos = armSittingPos;
    wait(300, msec);
    ladderLockPos = ladderSittingPos;
    /* ArmLift.rotateTo(armSittingPos, deg, 100, velocityUnits::pct, false);
    wait(100, msec);
    LadderLift.rotateTo(ladderSittingPos, deg, 75, velocityUnits::pct, false); */
  });

  Controller.ButtonA.pressed([]() {
    LadderLiftStopped = true;
    ladderLockPos = ladderExtended;
    // LadderLift.rotateTo(0, deg, 75, velocityUnits::pct, false);
    /* int ladderPos = LadderLift.rotation(deg);
    while (ladderPos < ladderExtended) {
      int LadderLiftSpeed = cos(ladderExtended - ladderPos) * 100;
      LadderLift.rotateFor(forward, 10, msec, LadderLiftSpeed, velocityUnits::pct);
      ladderPos = LadderLift.rotation(deg);
    } */
    // LadderLiftStopped = true;
  });

  /* PNEUMATICS */
  Controller.ButtonL2.pressed([](){
    PneumaticLeft.set(!PneumaticLeft.value());
    PneumaticRight.set(!PneumaticRight.value());
  });
}

int main() {
  vexcodeInit();
  
  comp.autonomous(autonomous);
  comp.drivercontrol(drivercontrol);
}
