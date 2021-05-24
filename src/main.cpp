/* 10703Z AUTONOMOUS AND DRIVER SKILLS CODE FOR LRS WORLD CHAMPIONSHIP */
#include "vex.h"
#include "robot-config.h"

using namespace vex;

competition    Competiton;

/* INITIALIZATIONS FOR THE NEEDED GLOBAL VARIABLES AND CONSTANTS */

// ball positioning constants, variables, and arrays
double distancePos0Unoccupied = 285; // in millimeters
double distancePos0EnteringConveyor = 140; // in millimeters
double distancePos0Occupied = 165; // in millimeters

double distancePos1Unoccupied = 170; // in millimeters
double distancePos1Occupied = 25; // in millimeters

double distancePos2Unoccupied = 190; // in millimeters
double distancePos2Occupied = 30; // in millimeters

double distanceScoringOccupied = 85; // in millimeters
double distanceScoringBallSeen = 100; // in millimeters

bool positionOneHalfOccupied; // ball part-way between position zero and position one
bool correctPositionOneHalf;

bool evaluated = false;
bool positionZeroOccupied, positionOneOccupied, positionTwoOccupied;
int positionArray[4] = {positionZeroOccupied, positionOneOccupied, positionTwoOccupied, evaluated};

bool positionZeroOccupiedNoFlags, positionOneOccupiedNoFlags, positionTwoOccupiedNoFlags;
int positionArrayNoFlags[3] = {positionZeroOccupiedNoFlags, positionOneOccupiedNoFlags, positionTwoOccupiedNoFlags};

// flags used in autonomous with brain screen selector and correction code
bool score, descore, descoreCorrection;

// used in autonomous functions for drivetrain movement, acceleration, deceleration, and drift correction
double accelerometerDriftThreshold = 0.02; // in g's (4.0 g's is the max returned by the inertial sensor)
bool accel, decel, fullSpeed;
double highestEncoderValue, leftDriftOffset, rightDriftOffset, leftEncoderAverage, 
rightEncoderAverage, driftError, currentDegree, currentAngle, initialAngle, degreesTurned, 
totalDegreesToTurn, degreesToAccel, degreesToDecel, degreesRemaining, percentAccelComplete, 
percentDecelComplete, setSpeed;

// counters, auto-sorting, ball positioning, and macro constants and variables
int count = 0, routineIndex = 0;
bool redAlliance, autoSort;
int pos1Status, pos2Status;
bool scoreTwo, conditionOne = false, conditionTwo = false;
bool wrongColor, allowManualIntakingWhileScoring;

/* BASIC FUNCTIONS FOR EACH NEEDED MOTOR MOVEMENT. used in autonomous and usercontrol */

void drivetrainStop() { // stops all four wheels
  DriveMotorLeftFront.stop(brakeType::brake); 
  DriveMotorLeftBack.stop(brakeType::brake);
  DriveMotorRightFront.stop(brakeType::brake);
  DriveMotorRightBack.stop(brakeType::brake);
}

void intake() { // spin both intakes inward at full speed
  IntakeMotorLeft.spin(directionType::fwd,200,velocityUnits::rpm);
  IntakeMotorRight.spin(directionType::fwd,200,velocityUnits::rpm);
}
void outtake(double velocity) { // spin both intakes outward at the specified speed
  IntakeMotorLeft.spin(directionType::rev,velocity,velocityUnits::rpm);
  IntakeMotorRight.spin(directionType::rev,velocity,velocityUnits::rpm);
}
void intakesStop() { // lock both intakes
  IntakeMotorLeft.stop(hold);
  IntakeMotorRight.stop(hold);
}

void conveyorSpin(bool forward, double velocity) { // spin the bottom three rollers at the specified speed and direction
  if(forward == true) {
    ConveyorMotor.spin(directionType::fwd,velocity,velocityUnits::rpm);
  }
  else if(forward == false) {
    ConveyorMotor.spin(directionType::rev,velocity,velocityUnits::rpm);
  }
}
void conveyorStop() { // stop the bottom three rollers. brakeType allows a minimal amount of deceleration
  ConveyorMotor.stop(brake);
}
void conveyorLock() { // lock the bottom three rollers
  ConveyorMotor.stop(hold);
}

void sorterSpin(bool forward, double velocity) { // spin the top two rollers at the specified speed and direction
  if(forward == true) {
    SortingMotor.spin(directionType::fwd,velocity,velocityUnits::rpm);
  }
  else if(forward == false) {
    SortingMotor.spin(directionType::rev,velocity,velocityUnits::rpm);
  }
}
void sorterScore() { // spin the top two rollers upward using volts for extra thrust
  SortingMotor.spin(directionType::fwd,12,voltageUnits::volt);
}
void sorterStop() { // stop the top two rollers. brakeType allows a minimal amount of deceleration
  SortingMotor.stop(brake);
}
void sorterLock() { // lock the top two rollers
  SortingMotor.stop(hold);
}

/* AUTO-SORTING AND BALL POSITIONING CODE. used in autonomous and usercontrol */

void autoSortToggle() { // toggle between red, blue, and off
  // if automatic sorting is currently set to off, then turn automatic sorting on and set to red
  if(autoSort == false) {
    redAlliance = true; 
    autoSort = true;
    Controller.Screen.setCursor(4, 1);
    Controller.Screen.print("RED           ");
  }
  else if(autoSort == true) {
    // if automatic sorting is currently on and set to red, then set it to blue
    if(redAlliance == true) {
      redAlliance = false;
      Controller.Screen.setCursor(4, 1);
      Controller.Screen.print("BLUE          ");
    }
    // if automatic sorting is currently on and set to blue, then turn automatic sorting off
    else if(redAlliance == false) {
      autoSort = false;
      Controller.Screen.setCursor(4, 1);
      Controller.Screen.print("OFF           ");
    } 
  }
  wait(200, msec); // reduces the toggle's level of reaction sensitivity
}

void opticalLightOn() { // turn the LEDs of both optical sensors on
  OpticalPos1.setLightPower(35, percent);
  OpticalPos1.setLight(ledState::on);
  OpticalPos2.setLightPower(35, percent);
  OpticalPos2.setLight(ledState::on);
}
void opticalLightOff() { // turn the LEDs of both optical sensors off
  OpticalPos1.setLight(ledState::off);
  OpticalPos2.setLight(ledState::off);
}

/* identifyColor is used with evaluatePosition, and identifyColorNoFlags is used with evaluatePositionNoFlags. 
The only difference is the array used and the flags changed. The NoFlags functions can be called anywhere, 
whereas the flags in the evaluatePosition function cannot be changed without messing up the flow of the macros. */

int evaluatePosition(int positionArray[4]) { // evaluates the positioning of balls in the conveyor and intakes
  // evaluate position zero (in the intakes)
  if(DistancePos0.objectDistance(mm) < distancePos0Unoccupied) {
    positionArray[0] /* positionZeroOccupied */ = true;
  }
  else {
    positionArray[0] /* positionZeroOccupied */ = false;
  }
  // evaluate position one (lower position in the conveyor)
  if(DistancePos1.objectDistance(mm) < distancePos1Unoccupied) {
    positionArray[1] /* positionOneOccupied */ = true;
  }
  else {
    positionArray[1] /* positionOneOccupied */ = false;
  }
  // evaluate position two (higher position in the conveyor)
  if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
    positionArray[2] /* positionTwoOccupied */ = true;
  }
  else {
    positionArray[2] /* positionTwoOccupied */ = false;
  }
  positionArray[3] /* evaluated */ = true; // flag that the ball positionings are current
  return positionArray[4]; // return the evaluation of each position
}
int evaluatePositionNoFlags(int positionArrayNoFlags[3]) { // evaluates the positioning of balls in the conveyor and intakes
  // evaluate position zero (in the intakes)
  if(DistancePos0.objectDistance(mm) < distancePos0Unoccupied) {
    positionArrayNoFlags[0] /* positionZeroOccupiedNoFlags */ = true;
  }
  else {
    positionArrayNoFlags[0] /* positionZeroOccupiedNoFlags */ = false;
  }
  // evaluate position one (lower position in the conveyor)
  if(DistancePos1.objectDistance(mm) < distancePos1Unoccupied) {
    positionArrayNoFlags[1] /* positionOneOccupiedNoFlags */ = true;
  }
  else {
    positionArrayNoFlags[1] /* positionOneOccupiedNoFlags */ = false;
  }
  // evaluate position two (higher position in the conveyor)
  if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
    positionArrayNoFlags[2] /* positionTwoOccupiedNoFlags */ = true;
  }
  else {
    positionArrayNoFlags[2] /* positionTwoOccupiedNoFlags */ = false;
  }
  return positionArrayNoFlags[3]; // return the evaluation of each position
}

void identifyColor() { // returns whether to keep / score or dispose based on ball positioning
  if(autoSort == true) { // only executes if automatic sorting is toggled on (either red or blue)
    // get the color of each optical sensor
    int ballPos1Color = OpticalPos1.color();
    int ballPos2Color = OpticalPos2.color();

    // reset flags
    bool colorPos1Identified = false;
    bool colorPos2Identified = false;
    int numberOfChecks = 0;

    /* Check for color based on the ball positioning to maximize efficiency */

    // if true-false (only one ball, in position one)
    if(positionArray[1] == true && positionArray[2] == false) {
      /* for one-fifth of a second, or until the ball in position one is identified as either red or blue, 
      keep checking the color */
      while(numberOfChecks <= 20) {
        wait(10, msec);
        ballPos1Color = OpticalPos1.color();
        if(ballPos1Color == red || ballPos1Color == blue) {
          colorPos1Identified = true;
          break; // ends the loop as soon as either red or blue is identified
        }
        numberOfChecks += 1; // increment the count
      }

      // if either red or blue was identified
      if(colorPos1Identified == true) {
        if(ballPos1Color == red && redAlliance == true) {
          pos1Status = 0; // keep or score if both the ball and alliance are red
        }
        else if(ballPos1Color == blue && redAlliance == true) {
          pos1Status = 1; // dispose if the ball is blue and alliance is red
        }
        else if(ballPos1Color == red && redAlliance == false) {
          pos1Status = 1; // dispose if the ball is red and alliance is blue
        }
        else if(ballPos1Color == blue && redAlliance == false) {
          pos1Status = 0; // keep or score if both the ball and alliance are blue
        }
      }
      // if neither red nor blue was identified after checking the full the one-fifth second
      else if(colorPos1Identified == false) {
        pos1Status = 2; // unknown
        Controller.rumble("-"); // vibrate the controller as an alert that the color was not able to be identified
      }
    }
    // if false-true (only one ball, in position two)
    else if(positionArray[1] == false && positionArray[2] == true) {
      /* for one-fifth of a second, or until the ball in position two is identified as either red or blue, 
      keep checking the color */
      while(numberOfChecks <= 20) {
        wait(10, msec);
        ballPos2Color = OpticalPos2.color();
        if(ballPos2Color == red || ballPos2Color == blue) {
          colorPos2Identified = true;
          break; // ends the loop as soon as either red or blue is identified
        }
        numberOfChecks += 1; // increment the count
      }

      // if either red or blue was identified
      if(colorPos2Identified == true) {
        if(ballPos2Color == red && redAlliance == true) {
          pos2Status = 0; // keep or score if both the ball and alliance are red
        }
        else if(ballPos2Color == blue && redAlliance == true) {
          pos2Status = 1; // dispose if the ball is blue and alliance is red
        }
        else if(ballPos2Color == red && redAlliance == false) {
          pos2Status = 1; // dispose if the ball is red and alliance is blue
        }
        else if(ballPos2Color == blue && redAlliance == false) {
          pos2Status = 0; // keep or score if both the ball and alliance are blue
        }
      }
      // if neither red nor blue was identified after checking the full the one-fifth second
      else if(colorPos2Identified == false) {
        pos2Status = 2; // unknown
        Controller.rumble("-"); // vibrate the controller as an alert that the color was not able to be identified
      }
    }
    // if true-true (two balls, in positions one and two)
    else if(positionArray[1] == true && positionArray[2] == true) {
      /* for one-fifth of a second, or until both balls are identified as either red or blue, 
      keep checking the colors */
      while(numberOfChecks <= 20) {
        wait(10, msec);

        if(colorPos1Identified == false) { // only check if the position one ball color has not already been identified
          ballPos1Color = OpticalPos1.color();
        }
        if(ballPos1Color == red || ballPos1Color == blue) {
          colorPos1Identified = true;
        }
        if(colorPos2Identified == false) { // only check if the position two ball color has not already been identified
          ballPos2Color = OpticalPos2.color();
        }
        if(ballPos2Color == red || ballPos2Color == blue) {
          colorPos2Identified = true;
        }
        if(colorPos1Identified == true && colorPos2Identified == true) {
          break; // ends the loop as soon as both balls are identified as either red or blue
        }
        numberOfChecks += 1; // increment the count
      }

      // if position one ball color was identified as either red or blue
      if(colorPos1Identified == true) {
        if(ballPos1Color == red && redAlliance == true) {
          pos1Status = 0; // keep or score the position one ball if both the ball and alliance are red
        }
        else if(ballPos1Color == blue && redAlliance == true) {
          pos1Status = 1; // dispose the position one ball if the ball is blue and alliance is red
        }
        else if(ballPos1Color == red && redAlliance == false) {
          pos1Status = 1; // dispose the position one ball if the ball is red and alliance is blue
        }
        else if(ballPos1Color == blue && redAlliance == false) {
          pos1Status = 0; // keep or score the position one ball if both the ball and alliance are blue
        }
      }
      // if the position one ball was unable to be identified after checking the full the one-fifth second
      else if(colorPos1Identified == false) {
        pos1Status = 2; // unknown
        Controller.rumble("-"); // vibrate the controller as an alert that the color was not able to be identified
      }

      // if position two ball color was identified as either red or blue
      if(colorPos2Identified == true) {
        if(ballPos2Color == red && redAlliance == true) {
          pos2Status = 0; // keep or score the position two ball if both the ball and alliance are red
        }
        else if(ballPos2Color == blue && redAlliance == true) {
          pos2Status = 1; // dispose the position two ball if the ball is blue and alliance is red
        }
        else if(ballPos2Color == red && redAlliance == false) {
          pos2Status = 1; // dispose the position two ball if the ball is red and alliance is blue
        }
        else if(ballPos2Color == blue && redAlliance == false) {
          pos2Status = 0; // keep or score the position two ball if both the ball and alliance are blue
        }
      }
      // if the position two ball was unable to be identified after checking the full the one-fifth second
      else if(colorPos2Identified == false) {
        pos2Status = 2; // unknown
        Controller.rumble("-"); // vibrate the controller as an alert that the color was not able to be identified
      }
    }
  }
  // if automatic sorting is toggled off, never automatically dispose
  else if(autoSort == false) { 
    pos1Status = 0; // keep or score
    pos2Status = 0; // keep or score
  }
}
void identifyColorNoFlags() { // returns whether to keep / score or dispose based on ball positioning
  if(autoSort == true) { // only executes if automatic sorting is toggled on (either red or blue)
    // get the color of each optical sensor
    int ballPos1Color = OpticalPos1.color();
    int ballPos2Color = OpticalPos2.color();

    // reset flags
    bool colorPos1Identified = false;
    bool colorPos2Identified = false;
    int numberOfChecks = 0;

    /* Check for color based on the ball positioning to maximize efficiency */

    // if true-false (only one ball, in position one)
    if(positionArrayNoFlags[1] == true && positionArrayNoFlags[2] == false) {
      /* for one-fifth of a second, or until the ball in position one is identified as either red or blue, 
      keep checking the color */
      while(numberOfChecks <= 20) {
        wait(10, msec);
        ballPos1Color = OpticalPos1.color();
        if(ballPos1Color == red || ballPos1Color == blue) {
          colorPos1Identified = true;
          break; // ends the loop as soon as either red or blue is identified
        }
        numberOfChecks += 1; // increment the count
      }

      // if either red or blue was identified
      if(colorPos1Identified == true) {
        if(ballPos1Color == red && redAlliance == true) {
          pos1Status = 0; // keep or score if both the ball and alliance are red
        }
        else if(ballPos1Color == blue && redAlliance == true) {
          pos1Status = 1; // dispose if the ball is blue and alliance is red
        }
        else if(ballPos1Color == red && redAlliance == false) {
          pos1Status = 1; // dispose if the ball is red and alliance is blue
        }
        else if(ballPos1Color == blue && redAlliance == false) {
          pos1Status = 0; // keep or score if both the ball and alliance are blue
        }
      }
      // if neither red nor blue was identified after checking the full the one-fifth second
      else if(colorPos1Identified == false) {
        pos1Status = 2; // unknown
        Controller.rumble("-"); // vibrate the controller as an alert that the color was not able to be identified
      }
    }
    // if false-true (only one ball, in position two)
    else if(positionArrayNoFlags[1] == false && positionArrayNoFlags[2] == true) {
      /* for one-fifth of a second, or until the ball in position two is identified as either red or blue, 
      keep checking the color */
      while(numberOfChecks <= 20) {
        wait(10, msec);
        ballPos2Color = OpticalPos2.color();
        if(ballPos2Color == red || ballPos2Color == blue) {
          colorPos2Identified = true;
          break; // ends the loop as soon as either red or blue is identified
        }
        numberOfChecks += 1; // increment the count
      }

      // if either red or blue was identified
      if(colorPos2Identified == true) {
        if(ballPos2Color == red && redAlliance == true) {
          pos2Status = 0; // keep or score if both the ball and alliance are red
        }
        else if(ballPos2Color == blue && redAlliance == true) {
          pos2Status = 1; // dispose if the ball is blue and alliance is red
        }
        else if(ballPos2Color == red && redAlliance == false) {
          pos2Status = 1; // dispose if the ball is red and alliance is blue
        }
        else if(ballPos2Color == blue && redAlliance == false) {
          pos2Status = 0; // keep or score if both the ball and alliance are blue
        }
      }
      // if neither red nor blue was identified after checking the full the one-fifth second
      else if(colorPos2Identified == false) {
        pos2Status = 2; // unknown
        Controller.rumble("-"); // vibrate the controller as an alert that the color was not able to be identified
      }
    }
    // if true-true (two balls, in positions one and two)
    else if(positionArrayNoFlags[1] == true && positionArrayNoFlags[2] == true) {
      /* for one-fifth of a second, or until both balls are identified as either red or blue, 
      keep checking the colors */
      while(numberOfChecks <= 20) {
        wait(10, msec);

        if(colorPos1Identified == false) { // only check if the position one ball color has not already been identified
          ballPos1Color = OpticalPos1.color();
        }
        if(ballPos1Color == red || ballPos1Color == blue) {
          colorPos1Identified = true;
        }
        if(colorPos2Identified == false) { // only check if the position two ball color has not already been identified
          ballPos2Color = OpticalPos2.color();
        }
        if(ballPos2Color == red || ballPos2Color == blue) {
          colorPos2Identified = true;
        }
        if(colorPos1Identified == true && colorPos2Identified == true) {
          break; // ends the loop as soon as both balls are identified as either red or blue
        }
        numberOfChecks += 1; // increment the count
      }

      // if position one ball color was identified as either red or blue
      if(colorPos1Identified == true) {
        if(ballPos1Color == red && redAlliance == true) {
          pos1Status = 0; // keep or score the position one ball if both the ball and alliance are red
        }
        else if(ballPos1Color == blue && redAlliance == true) {
          pos1Status = 1; // dispose the position one ball if the ball is blue and alliance is red
        }
        else if(ballPos1Color == red && redAlliance == false) {
          pos1Status = 1; // dispose the position one ball if the ball is red and alliance is blue
        }
        else if(ballPos1Color == blue && redAlliance == false) {
          pos1Status = 0; // keep or score the position one ball if both the ball and alliance are blue
        }
      }
      // if the position one ball was unable to be identified after checking the full the one-fifth second
      else if(colorPos1Identified == false) {
        pos1Status = 2; // unknown
        Controller.rumble("-"); // vibrate the controller as an alert that the color was not able to be identified
      }

      // if position two ball color was identified as either red or blue
      if(colorPos2Identified == true) {
        if(ballPos2Color == red && redAlliance == true) {
          pos2Status = 0; // keep or score the position two ball if both the ball and alliance are red
        }
        else if(ballPos2Color == blue && redAlliance == true) {
          pos2Status = 1; // dispose the position two ball if the ball is blue and alliance is red
        }
        else if(ballPos2Color == red && redAlliance == false) {
          pos2Status = 1; // dispose the position two ball if the ball is red and alliance is blue
        }
        else if(ballPos2Color == blue && redAlliance == false) {
          pos2Status = 0; // keep or score the position two ball if both the ball and alliance are blue
        }
      }
      // if the position two ball was unable to be identified after checking the full the one-fifth second
      else if(colorPos2Identified == false) {
        pos2Status = 2; // unknown
        Controller.rumble("-"); // vibrate the controller as an alert that the color was not able to be identified
      }
    }
  }
  // if automatic sorting is toggled off, never automatically dispose
  else if(autoSort == false) {
    pos1Status = 0; // keep or score
    pos2Status = 0; // keep or score
  }
}

/* TRIGONOMETRIC CONSTANTS, VARIABLES, AND FUNCTIONS. used in autonomous only */

double approxPi = 3.14159265359;
double wheelDiameter = 3.25; // in inches
double wheelCircumference = wheelDiameter * approxPi;
double degreesToReverse, degreesToStrafe,  degreesToDrive, degreeToTurnTo, theta, 
angleAdjustment, correctLeft, angleAtGoal;

// used to move from the right corner goal on the home row to the center goal on the home row
void trigCalcCornerToCenter(double targetDistanceFromGoalParallelToHomeRow, double targetDistanceFromGoalPerpendicularToHomeRow) {
  // get the current angle and correct to the best angle for the trig calculations
  double angleAtGoal = Inertial.yaw();
  if(angleAtGoal < 120) {
    DriveMotorLeftBack.spin(directionType::fwd,75,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::rev,75,velocityUnits::rpm);
    waitUntil(Inertial.yaw() >= 120);
    angleAtGoal = Inertial.yaw();
  }
  else if(angleAtGoal > 120) {
    DriveMotorLeftBack.spin(directionType::rev,75,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::fwd,75,velocityUnits::rpm);
    waitUntil(Inertial.yaw() <= 120);
    angleAtGoal = Inertial.yaw();
  }
  drivetrainStop();

  // determine the "triangle angle" from the inertial sensor angle
  if(angleAtGoal < 0) {
    if(angleAtGoal < -90) {
      theta = fabs(angleAtGoal + 90);
    }
    else {
      theta = fabs(angleAtGoal);
    }
  }
  else if(angleAtGoal >= 0) {
    if(angleAtGoal < 90) {
      theta = angleAtGoal;
    }
    else {
      theta = angleAtGoal - 90;
    }
  }

  // find the sine of the "triangle angle" and convert to radians
  double sinInRad = sin(theta * (approxPi / 180));
  // solve for the hypotenuse (distanceToReverse) using sinInRad and the opposite side (targetDistanceFromGoalPerpendicularToHomeRow)
  double distanceToReverse = targetDistanceFromGoalPerpendicularToHomeRow / sinInRad;
  // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
  degreesToReverse = distanceToReverse * (360 / (wheelCircumference * sqrt(2)));

  // find the tangent of the "triangle angle" and convert to radians
  double tanInRad = tan(theta * (approxPi / 180));
  // solve for the adjacent (distanceReversedParallelToHomeRow) using tanInRad and the opposite side (targetDistanceFromGoalPerpendicularToHomeRow)
  double distanceReversedParallelToHomeRow = targetDistanceFromGoalPerpendicularToHomeRow / tanInRad;
  /* find how far the robot needs to strafe after reversing by taking the difference of the total distance to go parallel to the home row and the 
  distance already traveled parallel to the home row in reversing */
  double distanceToStrafe = targetDistanceFromGoalParallelToHomeRow - distanceReversedParallelToHomeRow;
  // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
  degreesToStrafe = distanceToStrafe * (360 / (wheelCircumference * sqrt(2)));
}

// used to move from the center goal on the home row to the ball between the left corner and the center goals on the home row
void trigCalcCenterToBall(double targetDistanceFromGoalParallelToHomeRow, double targetDistanceFromGoalPerpendicularToHomeRow) {
  // get the current angle and determine which direction to turn when correcting to avoid problems with the +/-180 threshold
  angleAtGoal = Inertial.yaw();
  if(angleAtGoal < 0) {
    correctLeft = true;
    angleAdjustment = 180 + angleAtGoal;
  }
  else if(angleAtGoal > 0) {
    correctLeft = false;
    angleAdjustment = 180 - angleAtGoal;
  }

  // correct to the best angle for the trig calculations
  if(correctLeft == true && angleAtGoal > -179) {
    DriveMotorLeftBack.spin(directionType::rev,50,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::fwd,50,velocityUnits::rpm);
    waitUntil(Inertial.yaw() <= -179);
    angleAtGoal = Inertial.yaw();
    angleAdjustment = 180 + angleAtGoal;
  }
  else if(correctLeft == false && angleAtGoal < 179) {
    DriveMotorLeftBack.spin(directionType::fwd,50,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::rev,50,velocityUnits::rpm);
    waitUntil(Inertial.yaw() >= 179);
    angleAtGoal = Inertial.yaw();
    angleAdjustment = 180 - angleAtGoal;
  }
  drivetrainStop();

  // find the cosine and tangent of the "triangle angle" and convert to radians
  double cosAngleAdjustmentInRad = cos(angleAdjustment * (approxPi / 180));
  double tanAngleAdjustmentInRad = tan(angleAdjustment * (approxPi / 180));

  // solve for the hypotenuse (distanceToReverse) using cosAngleAdjustmentInRad and the adjacent side (targetDistanceFromGoalPerpendicularToHomeRow)
  double distanceToReverse = targetDistanceFromGoalPerpendicularToHomeRow / cosAngleAdjustmentInRad;
  // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
  degreesToReverse = distanceToReverse * (360 / (wheelCircumference * sqrt(2)));

  // solve for the opposite (distanceToReverse) using tanAngleAdjustmentInRad and the adjacent side (targetDistanceFromGoalPerpendicularToHomeRow)
  double distanceReversedParallelToHomeRow = tanAngleAdjustmentInRad * targetDistanceFromGoalPerpendicularToHomeRow;

  // based on the angle at the goal after correction, either add or subtract to account for the distance traveled when reversing at that angle
  if(correctLeft == true) {
    double distanceToStrafe = targetDistanceFromGoalParallelToHomeRow + distanceReversedParallelToHomeRow;
    // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
    degreesToStrafe = distanceToStrafe * (360 / (wheelCircumference * sqrt(2)));
  }
  else if(correctLeft == false) {
    double distanceToStrafe = targetDistanceFromGoalParallelToHomeRow - distanceReversedParallelToHomeRow;
    // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
    degreesToStrafe = distanceToStrafe * (360 / (wheelCircumference * sqrt(2)));
  }
}

// used to move from the left corner on the home row to the ball on the autonomous line nearest the left side goal on the center row
void trigCalcSecondCornerToBall(double targetDistanceFromGoalParallelToHomeRow, double targetDistanceFromGoalPerpendicularToHomeRow) {
  // specify how much of the total distance perpendicular you want to be reversing, and consequently how much should be driving
  double distanceTraveledPerpendicularToHomeRowWhileReversing = targetDistanceFromGoalPerpendicularToHomeRow / 2;
  // use the Pythagorean theorem to find the length of the hypotenuse (distanceToReverse)
  double distanceToReverse = sqrt(pow(targetDistanceFromGoalParallelToHomeRow, 2) + 
  pow(distanceTraveledPerpendicularToHomeRowWhileReversing, 2));
  // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
  degreesToReverse = distanceToReverse * (360 / (wheelCircumference * sqrt(2)));

  /* find how far the robot needs to drive after reversing by taking the difference of the total distance to go perpendicular 
  to the home row and the distance already traveled perpendicular to the home row in reversing */
  double distanceToDrive = targetDistanceFromGoalPerpendicularToHomeRow - 
  distanceTraveledPerpendicularToHomeRowWhileReversing;
  // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
  degreesToDrive = distanceToDrive * (360 / (wheelCircumference * sqrt(2)));

  /* solve for the angle at the goal needed for the previous calculations to work using inverse tangent and convert 
  to "triangle angle" degrees from radians */
  double targetAngleAtGoal = atan(targetDistanceFromGoalParallelToHomeRow / 
  distanceTraveledPerpendicularToHomeRowWhileReversing) * (180 / approxPi);

  // determine the "triangle angle" degrees from the current angle at the goal
  double angleAtGoal = 180 - fabs(Inertial.yaw());

  // correct the angle at the goal by comparing the current "triangle angle" at the goal to the target "triangle angle"
  if(angleAtGoal < targetAngleAtGoal) {
    DriveMotorLeftBack.spin(directionType::fwd,75,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::rev,75,velocityUnits::rpm);
    waitUntil(180 - fabs(Inertial.yaw()) >= targetAngleAtGoal);
  }
  else if(angleAtGoal > targetAngleAtGoal) {
    DriveMotorLeftBack.spin(directionType::rev,75,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::fwd,75,velocityUnits::rpm);
    waitUntil(180 - fabs(Inertial.yaw()) <= targetAngleAtGoal);
  }
  drivetrainStop();
}

// used to move from the far left corner goal on the far home row to the ball between the middle goal and the far center goal on the far home row
void trigCalcThirdCornerToBall(double targetDistanceFromGoalParallelToHomeRow, double targetDistanceFromGoalPerpendicularToHomeRow) {
  // get the current angle and correct to the best angle for the trig calculations
  double angleAtGoal = fabs(Inertial.yaw());
  if(angleAtGoal > 35) {
    DriveMotorLeftBack.spin(directionType::fwd,50,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::rev,50,velocityUnits::rpm);
    waitUntil(fabs(Inertial.yaw()) <= 35);
    angleAtGoal = fabs(Inertial.yaw());
  }
  else if(angleAtGoal < 35) {
    DriveMotorLeftBack.spin(directionType::rev,50,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::fwd,50,velocityUnits::rpm);
    waitUntil(fabs(Inertial.yaw()) >= 35);
    angleAtGoal = fabs(Inertial.yaw());
  }
  drivetrainStop();
  angleAtGoal = 90 - angleAtGoal;

  // find the sine of the "triangle angle" and convert to radians
  double sinInRad = sin(angleAtGoal * (approxPi / 180));
  // solve for the hypotenuse (distanceToReverse) using sinInRad and the opposite side (targetDistanceFromGoalPerpendicularToHomeRow)
  double distanceToReverse = targetDistanceFromGoalPerpendicularToHomeRow / sinInRad;
  // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
  degreesToReverse = distanceToReverse * (360 / (wheelCircumference * sqrt(2)));

  // find the tangent of the "triangle angle" and convert to radians
  double tanInRad = tan(angleAtGoal * (approxPi / 180));
  // solve for the adjacent (distanceReversedParallelToHomeRow) using tanInRad and the opposite side (targetDistanceFromGoalPerpendicularToHomeRow)
  double distanceReversedParallelToHomeRow = targetDistanceFromGoalPerpendicularToHomeRow / tanInRad;
  /* find how far the robot needs to strafe after reversing by taking the difference of the total distance to go parallel to the home row and the 
  distance already traveled parallel to the home row in reversing */
  double distanceToDrive = targetDistanceFromGoalParallelToHomeRow - distanceReversedParallelToHomeRow;
  // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
  degreesToDrive = distanceToDrive * (360 / (wheelCircumference * sqrt(2)));
}

/* same as the trigCalcCenterToBall function but different angle evaluation due to the orientation of the inertial sensor 
when on the other side of the field */
void trigCalcSixthGoalToBall(double targetDistanceFromGoalParallelToHomeRow, double targetDistanceFromGoalPerpendicularToHomeRow) {
  // get the current angle and correct to the best angle for the trig calculations in the best direction to turn to avoid problems with the +/-180 threshold
  angleAtGoal = Inertial.yaw();
  if(angleAtGoal < -2) {
    correctLeft = false;
    DriveMotorLeftBack.spin(directionType::fwd,50,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::rev,50,velocityUnits::rpm);
    waitUntil(Inertial.yaw() >= -2);
    angleAtGoal = fabs(Inertial.yaw());
  }
  else if(angleAtGoal > 2) {
    correctLeft = true;
    DriveMotorLeftBack.spin(directionType::rev,50,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::fwd,50,velocityUnits::rpm);
    waitUntil(Inertial.yaw() <= 2);
    angleAtGoal = fabs(Inertial.yaw());
  }
  drivetrainStop();

  // find the cosine and tangent of the "triangle angle" and convert to radians
  double cosAngleAtGoalInRad = cos(angleAtGoal * (approxPi / 180));
  double tanAngleAtGoalInRad = tan(angleAtGoal * (approxPi / 180));

  // solve for the hypotenuse (distanceToReverse) using cosAngleAdjustmentInRad and the adjacent side (targetDistanceFromGoalPerpendicularToHomeRow)
  double distanceToReverse = targetDistanceFromGoalPerpendicularToHomeRow / cosAngleAtGoalInRad;
  // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
  degreesToReverse = distanceToReverse * (360 / (wheelCircumference * sqrt(2)));

  // solve for the opposite (distanceToReverse) using tanAngleAdjustmentInRad and the adjacent side (targetDistanceFromGoalPerpendicularToHomeRow)
  double distanceReversedParallelToHomeRow = tanAngleAtGoalInRad * targetDistanceFromGoalPerpendicularToHomeRow;

  // based on the angle at the goal after correction, either add or subtract to account for the distance traveled when reversing at that angle
  if(correctLeft == true) {
    double distanceToStrafe = targetDistanceFromGoalParallelToHomeRow + distanceReversedParallelToHomeRow;
    // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
    degreesToStrafe = distanceToStrafe * (360 / (wheelCircumference * sqrt(2)));
  }
  else if(correctLeft == false) {
    double distanceToStrafe = targetDistanceFromGoalParallelToHomeRow - distanceReversedParallelToHomeRow;
    // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
    degreesToStrafe = distanceToStrafe * (360 / (wheelCircumference * sqrt(2)));
  }
}

/* same as the trigCalcSecondCornerToBall function but different angle evaluation due to the orientation of the inertial sensor 
when on the other side of the field */
void trigCalcFourthCornerToBall(double targetDistanceFromGoalParallelToHomeRow, double targetDistanceFromGoalPerpendicularToHomeRow) {
  // specify how much of the total distance perpendicular you want to be reversing, and consequently how much should be driving
  double distanceTraveledPerpendicularToHomeRowWhileReversing = targetDistanceFromGoalPerpendicularToHomeRow / 2;
  // use the Pythagorean theorem to find the length of the hypotenuse (distanceToReverse)
  double distanceToReverse = sqrt(pow(targetDistanceFromGoalParallelToHomeRow, 2) + 
  pow(distanceTraveledPerpendicularToHomeRowWhileReversing, 2));
  // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
  degreesToReverse = distanceToReverse * (360 / (wheelCircumference * sqrt(2)));

  /* find how far the robot needs to drive after reversing by taking the difference of the total distance to go perpendicular 
  to the home row and the distance already traveled perpendicular to the home row in reversing */
  double distanceToDrive = targetDistanceFromGoalPerpendicularToHomeRow - 
  distanceTraveledPerpendicularToHomeRowWhileReversing;
  // convert inches to encoder degrees. sqrt(2) accounts for the X-Drive
  degreesToDrive = distanceToDrive * (360 / (wheelCircumference * sqrt(2)));

  /* solve for the angle at the goal needed for the previous calculations to work using inverse tangent and convert 
  to degrees from radians */
  double targetAngleAtGoal = atan(targetDistanceFromGoalParallelToHomeRow / 
  distanceTraveledPerpendicularToHomeRowWhileReversing) * (180 / approxPi);

  // correct the angle at the goal by comparing the current angle at the goal to the target angle
  double angleAtGoal = Inertial.yaw();
  if(angleAtGoal < targetAngleAtGoal) {
    DriveMotorLeftBack.spin(directionType::fwd,75,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::rev,75,velocityUnits::rpm);
    waitUntil(Inertial.yaw() >= targetAngleAtGoal);
  }
  else if(angleAtGoal > targetAngleAtGoal) {
    DriveMotorLeftBack.spin(directionType::rev,75,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::fwd,75,velocityUnits::rpm);
    waitUntil(Inertial.yaw() <= targetAngleAtGoal);
  }
  drivetrainStop();
}

/* USERCONTROL MACROS AND MANUAL CONTROL FUNCTIONS */

int drivetrainMotion(int drivetrainArray[3]) {
  int joystickStopRange = 3; /* this specifies the range of values returning from the 
  Controller.Axis.value() function in which the wheels are signaled to brake (a broader 
  range prevents the robot from continuing to move if the joystick isn't exactly at 0) */
  int speedControlThreshold = 105; // threshold to use rpm or volts for slower or high speed

  // if the joystick axis for turning is above the threshold to stop. gives turning priority over driving and strafing
  if(abs(Controller.Axis4.value()) > joystickStopRange) {
    drivetrainArray[0] /* driveActive */ = false;
    drivetrainArray[1] /* turnActive */ = true;
    drivetrainArray[2] /* strafeActive*/ = false;
  }
  // if the joystick axis for driving and / or strafing is above the threshold to stop
  else if(abs(Controller.Axis2.value()) > joystickStopRange 
  || abs(Controller.Axis1.value()) > joystickStopRange) {
    
    // determines whether driving or strafing has a higher value
    if(abs(Controller.Axis2.value()) > abs(Controller.Axis1.value())) {
      drivetrainArray[0] /* driveActive */ = true;
      drivetrainArray[1] /* turnActive */ = false;
      drivetrainArray[2] /* strafeActive*/ = false;
    }
    else if(abs(Controller.Axis2.value()) <= abs(Controller.Axis1.value())) {
      drivetrainArray[0] /* driveActive */ = false;
      drivetrainArray[1] /* turnActive */ = false;
      drivetrainArray[2] /* strafeActive*/ = true;
    }
  }
  // if none of the joystick axes are above the threshold to stop
  else {
    drivetrainArray[0] /* driveActive */ = false;
    drivetrainArray[1] /* turnActive */ = false;
    drivetrainArray[2] /* strafeActive*/ = false;
    drivetrainStop();
  }

  // if driving is active
  if(drivetrainArray[0] /* driveActive */ == true) {
    // Axis2 is y-axis on right joystick, controls straight forward and backward drivetrain motion
    if(abs(Controller.Axis2.value()) < speedControlThreshold) { // using rpm to allow slower movement
      DriveMotorLeftFront.spin(directionType::fwd,Controller.Axis2.value(),velocityUnits::rpm);
      DriveMotorLeftBack.spin(directionType::fwd,Controller.Axis2.value(),velocityUnits::rpm);
      DriveMotorRightFront.spin(directionType::fwd,Controller.Axis2.value(),velocityUnits::rpm);
      DriveMotorRightBack.spin(directionType::fwd,Controller.Axis2.value(),velocityUnits::rpm);
    }
    else { // using volts for higher speed
      DriveMotorLeftFront.spin(directionType::fwd,Controller.Axis2.value(),voltageUnits::volt);
      DriveMotorLeftBack.spin(directionType::fwd,Controller.Axis2.value(),voltageUnits::volt);
      DriveMotorRightFront.spin(directionType::fwd,Controller.Axis2.value(),voltageUnits::volt);
      DriveMotorRightBack.spin(directionType::fwd,Controller.Axis2.value(),voltageUnits::volt);
    }
  }    

  // if turning is active
  else if(drivetrainArray[1] /* turnActive */ == true) {
    // Axis4 is x-axis on left joystick, controls drivetrain turning left and right
    if(abs(Controller.Axis4.value()) < speedControlThreshold) { // using rpm to allow slower movement
      DriveMotorLeftFront.spin(directionType::fwd,Controller.Axis4.value(),velocityUnits::rpm);
      DriveMotorLeftBack.spin(directionType::fwd,Controller.Axis4.value(),velocityUnits::rpm);
      DriveMotorRightFront.spin(directionType::rev,Controller.Axis4.value(),velocityUnits::rpm);
      DriveMotorRightBack.spin(directionType::rev,Controller.Axis4.value(),velocityUnits::rpm);
    }
    else { // using volts for higher speed
      DriveMotorLeftFront.spin(directionType::fwd,Controller.Axis4.value(),voltageUnits::volt);
      DriveMotorLeftBack.spin(directionType::fwd,Controller.Axis4.value(),voltageUnits::volt);
      DriveMotorRightFront.spin(directionType::rev,Controller.Axis4.value(),voltageUnits::volt);
      DriveMotorRightBack.spin(directionType::rev,Controller.Axis4.value(),voltageUnits::volt);
    }
  }

  // if strafing is active
  else if(drivetrainArray[2] /* strafeActive */ == true) {
    // Axis1 is x-axis on right joystick, controls drivetrain strafing left and right
    if(abs(Controller.Axis1.value()) < speedControlThreshold) { // using rpm to allow slower movement
      DriveMotorLeftFront.spin(directionType::fwd,Controller.Axis1.value(),velocityUnits::rpm);
      DriveMotorLeftBack.spin(directionType::rev,Controller.Axis1.value(),velocityUnits::rpm);
      DriveMotorRightFront.spin(directionType::rev,Controller.Axis1.value(),velocityUnits::rpm);
      DriveMotorRightBack.spin(directionType::fwd,Controller.Axis1.value(),velocityUnits::rpm);
    }
    else { // using volts for higher speed
      DriveMotorLeftFront.spin(directionType::fwd,Controller.Axis1.value(),voltageUnits::volt);
      DriveMotorLeftBack.spin(directionType::rev,Controller.Axis1.value(),voltageUnits::volt);
      DriveMotorRightFront.spin(directionType::rev,Controller.Axis1.value(),voltageUnits::volt);
      DriveMotorRightBack.spin(directionType::fwd,Controller.Axis1.value(),voltageUnits::volt);
    }
  }
  return drivetrainArray[3]; // returns the values to use in the next iteration
}

int intakesManual(int intakesManualArray[1]) {
  // intake at full speed when ButtonR2 is being pressed
  if(Controller.ButtonR2.pressing() == true) {
    positionOneHalfOccupied = false; // reset flag used in collectBall macro
    intakesManualArray[0] = true;
    intake();
  }
  // outtake at full speed when ButtonLeft is being pressed
  else if(Controller.ButtonLeft.pressing() == true) {
    positionOneHalfOccupied = false; // reset flag used in collectBall macro
    intakesManualArray[0] = true;
    outtake(200);
  }
  // stop both intake motors when not in use
  if(intakesManualArray[0] == true && Controller.ButtonR2.pressing() == false && 
  Controller.ButtonLeft.pressing() == false) {
    intakesStop();
    intakesManualArray[0] = false;
  }
  return intakesManualArray[1];
}

int conveyorAndSorterManual(int conveyorAndSorterManualArray[1]) {
  // spin both the conveyor and sorter motors upward to score when ButtonUp is being pressed
  if(Controller.ButtonUp.pressing() == true) {
    positionOneHalfOccupied = false; // reset flag used in collectBall macro
    conveyorAndSorterManualArray[0] = true;
    sorterScore();
    conveyorSpin(true, 600);
  }
  // spin the conveyor motor upward and the sorter motor downward to dispose when ButtonRight is being pressed
  else if(Controller.ButtonRight.pressing() == true) {
    positionOneHalfOccupied = false; // reset flag used in collectBall macro
    conveyorAndSorterManualArray[0] = true;
    sorterSpin(false, 600);
    conveyorSpin(true, 600);
  }
  // spin both the conveyor and sorter motors downward to reverse when ButtonDown is being pressed
  else if(Controller.ButtonDown.pressing() == true) {
    positionOneHalfOccupied = false; // reset flag used in collectBall macro
    conveyorAndSorterManualArray[0] = true;
    sorterSpin(false, 600);
    conveyorSpin(false, 600);
  }
  // stop both the conveyor and sorter motors when not in use
  if(conveyorAndSorterManualArray[0] == true && Controller.ButtonUp.pressing() == false 
  && Controller.ButtonRight.pressing() == false && Controller.ButtonDown.pressing() == false) {
    conveyorStop();
    sorterStop();
    conveyorAndSorterManualArray[0] = false;
  }
  return conveyorAndSorterManualArray[1];
}

int collectBall(int collectBallArray[1]) {
  if(Controller.ButtonR1.pressing() == true && collectBallArray[0] == false) {
    /* "lock" all other macros and controls that use the intakes, conveyor, and sorter to prevent conflicting motor commands */
    collectBallArray[0] = true;
  }
  if(collectBallArray[0] == true) {
    // only evaluate the position on the first iteration. determine which section of code needs to be run
    if(positionArray[3] /* evaluated */ == false) {
      evaluatePosition(positionArray);
      /* check to see if the balls did not end up in the right place at the end of the last collectBall execution 
      (if any other macros or manual controls were initiated, the positionOneHalfOccupied flag will be reset to false) */
      if(positionOneHalfOccupied == true) {
        correctPositionOneHalf = true;
      }
      else {
        correctPositionOneHalf = false;
      }
    }

    /* correction for positionOneHalfOccupied. only executes if positionOneHalfOccupied is true on the first iteration.
    in case the higher ball reaches position two before the lower ball reaches position one */
    if(correctPositionOneHalf == true) {
      if(count == 0) { // reverse the conveyor to bring the ball in position two down
        sorterLock();
        conveyorSpin(false, 600);
        count = 1;
      }
      if(count == 1) {
        if(DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          positionOneHalfOccupied = true; 
          conveyorStop();
          intake(); // bring both balls up once the higher ball is out of position two
          conveyorSpin(true, 600);
          count = 2;
        }
      }
      if(count == 2) {
        if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
          intakesStop(); // stop once the higher ball reaches position two
          conveyorStop();
          positionOneHalfOccupied = false; // resets the flag so that this correction code cannot run multiple times in a row
          correctPositionOneHalf = false;
          collectBallArray[0] /* collectBallActive */ = false; // reset flags and counters
          count = 0;
          positionArray[3] /* evaluated */ = false;
        }
      }
    }

    /* false-false-false or true-false-false. only dealing with one ball, not initially in the conveyor */
    else if((positionArray[0] == false && positionArray[1] == false && positionArray[2] == false) || 
    (positionArray[0] == true && positionArray[1] == false && positionArray[2] == false)) {

      if(count == 0) {
        intake();
        opticalLightOn();
        count = 1;
      }
      if(count == 1) { // start the conveyor once the ball is in the intakes
        if(DistancePos0.objectDistance(mm) <= distancePos0Occupied) {
          conveyorSpin(true, 600);
          count = 2;
        }
      }
      if(count == 2) { // stop the intakes once ball is in the conveyor
        if(DistancePos1.objectDistance(mm) < distancePos1Occupied) {
          intakesStop();
          count = 3;
        }
      }
      if(count == 3) { // stop the conveyor once the ball is in position two
        if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
          conveyorStop();
          count = 4;
        }
      }
      if(count == 4) { // check positioning and color of ball
        evaluatePositionNoFlags(positionArrayNoFlags);
        identifyColorNoFlags();
        if(pos2Status == 1 /* dispose */) { // if the ball is the color of the opposing alliance
          conveyorSpin(true, 600);
          sorterSpin(false, 600);
          count = 5;
          wait(2, msec);
        }
        else if(pos2Status == 0 /* keep */ || pos2Status == 2 /* unknown */) {
          count = 6; // if the ball is the color of my alliance or was unable to be identified
        }
      }
      if(count == 5) {
        if(DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          conveyorStop(); // stop once ball has been disposed
          sorterStop();
          count = 6; // reset and exit
        }
      }
      if(count == 6) { // reset flags and counters
        collectBallArray[0] /* collectBallActive */ = false;
        count = 0;
        positionOneHalfOccupied = false;
        positionArray[3] /* evaluated */ = false;
        opticalLightOff();
      }
    }

    /* false-true-false or true-true-false. dealing with two balls, one initally in position one */
    else if((positionArray[0] == false && positionArray[1] == true && positionArray[2] == false) || 
    (positionArray[0] == true && positionArray[1] == true && positionArray[2] == false)) {

      if(count == 0) {
        intake();
        opticalLightOn();
        count = 1;
      }
      if(count == 1) { // start conveyor once the lower ball is in the intakes
        if(DistancePos0.objectDistance(mm) <= distancePos0EnteringConveyor) {
          conveyorSpin(true, 500);
          positionOneHalfOccupied = true;
          sorterLock();
          count = 2;
        }
      }
      if(count == 2) { // stop the intakes and conveyor once the higher ball reaches position two
        if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
          intakesStop();
          conveyorStop();
          positionOneHalfOccupied = false;
          count = 3;
        }
      }
      if(count == 3) { // check the positioning and color of both balls
        evaluatePositionNoFlags(positionArrayNoFlags);
        identifyColorNoFlags();
        if(pos2Status == 0 /* keep */) { // if the higher ball is the color of my alliance
          if(DistancePos1.objectDistance(mm) >= distancePos1Unoccupied) { // check that balls are positioned correctly
            positionOneHalfOccupied = true;
          }
          count = 9; // reset and exit
        }
        else if(pos1Status == 1 /* dispose */ && (pos2Status == 1 /* dispose */ || pos2Status == 2 /* unknown */)) {
          conveyorSpin(true, 600); // dispose both, both are the color of the opposing alliance
          sorterSpin(false, 600);
          count = 4;
          wait(2, msec);
        }
        else { // dispose one, the higher ball is the color of the opposing alliance, the lower ball is the color of my alliance
          conveyorSpin(true, 600);
          sorterSpin(false, 600);
          count = 7;
          wait(2, msec);
        }
      }
      if(count == 4) { // executes only for dispose both. check that one ball has been disposed
        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          count = 5;
        }
      }
      if(count == 5) { // check that the lower ball reaches position two
        if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
          count = 6;
        }
      }
      if(count == 6) { // stop once both balls have been disposed
        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          conveyorStop();
          sorterStop();
          count = 9; // reset and exit
        }
      }
      if(count == 7) { // only executes for dispose one
        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied) {
          sorterStop(); // stop once ball one is disposed
          count = 8;
        }
      }
      if(count == 8) {
        if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
          conveyorStop(); // stop once ball two is in position two
          count = 9; // reset and exit
        }
      }
      if(count == 9) { // reset flags and counters
        collectBallArray[0] /* collectBallActive */ = false;
        count = 0;
        positionArray[3] /* evaluated */ = false;
        opticalLightOff();
      }
    }

    /* false-false-true or true-false-true. dealing with two balls, one initially in position two */
    else if((positionArray[0] == false && positionArray[1] == false && positionArray[2] == true) || 
    (positionArray[0] == true && positionArray[1] == false && positionArray[2] == true)) {

      if(count == 0) {
        intake();
        conveyorSpin(false, 600); // bring ball in position two down to position one
        opticalLightOn();
        count = 1;
      }
      if(count == 1) { // stop once the ball from position two reaches position one and the second ball in the intakes
        if(DistancePos1.objectDistance(mm) < distancePos1Unoccupied) {
          conveyorLock();
          sorterLock();
        }
        if(DistancePos0.objectDistance(mm) <= distancePos0EnteringConveyor && DistancePos0.isObjectDetected() == true) {
          positionOneHalfOccupied = true;
          intakesStop();
          conveyorStop();
          sorterLock();
          count = 2; // will run true-true-false code next iteration (reduces duplicate code)
          positionArray[1] = true;
          positionArray[2] = false;
          intake(); // begin bringing both balls up
          conveyorSpin(true, 500);
        }
      }
    }

    /* false-true-true or true-true-true. dealing with three balls, two initially in positions one and two */
    else if((positionArray[0] == false && positionArray[1] == true && positionArray[2] == true) || 
    (positionArray[0] == true && positionArray[1] == true && positionArray[2] == true)) {

      if(count == 0) {
        intake();
        opticalLightOn();
        count = 1;
      }
      if(count == 1) { // stop once ball three is in the intakes
        if(DistancePos0.objectDistance(mm) <= distancePos0Occupied) {
          intakesStop();
          count = 2;
        }
      }
      if(count == 2) { // check the colors and positioning of the balls. must evaluate the third ball separately!
        identifyColor();
        if(pos2Status == 0 /* keep */) { // if the highest ball is the color of my alliance
          count = 9; // reset and exit
        }
        else if(pos1Status == 1 /* dispose */ && (pos2Status == 1 /* dispose */ || pos2Status == 2 /* unknown */)) {
          conveyorSpin(true, 600); // dispose both, both balls in the conveyor are the color of the opposing alliance
          sorterSpin(false, 600);
          count = 3;
          wait(2, msec);
        }
        else { // dispose one, the highest ball is the color of the opposing alliance, the second ball is the color of my alliance
          conveyorSpin(true, 600);
          sorterSpin(false, 600);
          count = 6;
          wait(2, msec);
        }
      }
      if(count == 3) { // only executes for dispose both. check that one ball has been disposed
        if(DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          count = 4;
        }
      }
      if(count == 4) { // check that the second ball reaches position two
        if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
          count = 5;
        }
      }
      if(count == 5) {
        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          conveyorStop(); // stop once both balls have been disposed
          sorterStop();
          evaluatePosition(positionArray); // will run true-false-false next iteration to check the third ball
          intake();
          conveyorSpin(true, 600);
          positionOneHalfOccupied = true;
          count = 2; 
        }
      }
      if(count == 6) { // only executes for dispose one
        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied) {
          conveyorStop(); // stop once the highest ball is disposed
          sorterStop();
          conveyorSpin(false, 600); // take the second ball back down to position one
          count = 7;
        }
      }
      if(count == 7) {
        if(DistancePos1.objectDistance(mm) < distancePos1Unoccupied) {
          conveyorLock(); // stop once the second ball reaches position one
          intake();
          conveyorSpin(true, 600); // bring both balls up to positions one and two
          positionOneHalfOccupied = true;
          count = 8;
        }
      }
      if(count == 8) {
        if(DistancePos1.objectDistance(mm) <= distancePos1Occupied 
        && DistancePos0.objectDistance(mm) > distancePos0EnteringConveyor) {
          intakesStop(); // stop the intakes once the third ball is in the conveyor
        }
        if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
          intakesStop(); // stop once the higher ball reaches position two
          conveyorStop();
          if(DistancePos1.objectDistance(mm) >= distancePos1Unoccupied) {
            positionOneHalfOccupied = true; // check that both balls reached the spots they were supposed to
          }
          else {
            positionOneHalfOccupied = false;
          }
          count = 9; // reset and exit
        }
      }
      if(count == 9) { // reset flags and counters
        collectBallArray[0] /* collectBallActive */ = false;
        count = 0;
        positionArray[3] /* evaluated */ = false;
        opticalLightOff();
      }
    }
  }
  return collectBallArray[1];
}

int scoreBall(int scoreBallArray[1]) {
  if(Controller.ButtonL1.pressing() == true && scoreBallArray[0] == false && scoreTwo == false) {
    /* the condition that scoreTwo == false is so that if L2 was pressed before L1, it does not evaluate
    again after being redirected from the scoreAndDescore macro */
  
    int numberOfChecks = 0; // reset flags
    scoreBallArray[0] = true; /* "lock" all other macros and controls that use the intakes, conveyor, and 
    sorter to prevent conflicting motor commands, except for the special flag allowManualIntakingWhileScoring 
    which allows the manual intake controls to be used while scoreBall is active in the sections where the 
    intakes are used in the macro */

    // for a tenth of a second, check to see if both L1 and L2 are being pressed at the same time, otherwise score one
    if(positionArray[3] /* evaluated */ == false) {
      while(numberOfChecks <= 25) {
        wait(4, msec);
        if(Controller.ButtonL2.pressing() == true) {
          scoreTwo = true;
          break;
        }
        numberOfChecks += 1;
      }
    }
  }
  if(scoreBallArray[0] == true) {
    // only evaluate the position on the first iteration. determine which section of code needs to be run
    if(positionArray[3] /* evaluated */ == false) {
      correctPositionOneHalf = false; // reset flag used in collectBall macro
      evaluatePosition(positionArray);
    }

    /* false-false. no balls initially in the conveyor */
    if(positionArray[1] == false && positionArray[2] == false) {
      if(allowManualIntakingWhileScoring == true) { // the intakes are used in this section, no manual control allowed
        allowManualIntakingWhileScoring = false;
        intakesStop();
      }
      if(count == 0) {
        opticalLightOn();
        count = 1;
      }
      if(count == 1) {
        intake();
        sorterScore();
        conveyorSpin(true, 600);
        count = 2;
      }
      if(count == 2) { // stop once ball is in the conveyor and reaches position two
        if(DistancePos1.objectDistance(mm) <= distancePos1Unoccupied) {
          intakesStop();
        }
        if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
          intakesStop();
          conveyorLock();
          count = 3;
        }
      }
      if(count == 3) { // check the color of the ball
        evaluatePositionNoFlags(positionArrayNoFlags);
        identifyColorNoFlags();
        if(pos2Status == 1 /* dispose */) { // if the ball is the color of the opposing alliance
          conveyorSpin(true, 600);
          sorterSpin(false, 600); // dispose
          count = 4;
          wait(2, msec);
        }
        else if(pos2Status == 0 /* score */ || pos2Status == 2 /* unknown */) { 
          sorterScore(); // if the ball is the color of my alliance
          conveyorSpin(true, 600); // score
          count = 5;
          wait(2, msec);
        }
      }
      if(count == 4) { // only executes if ball is to be disposed
        if(DistancePos2.objectDistance(mm) >= distancePos2Unoccupied) {
          conveyorStop(); // stop once ball is disposed
          sorterStop();
          count = 1; // descore and re-evaluate
        }
      }
      if(count == 5) { // only executes if ball is to be scored
        if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied) {
          count = 6; // check that the ball is in the process of being scored
        }
      }
      if(count == 6) {
        if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
          sorterStop(); // stop once ball is scored
          conveyorStop();
          count = 7;
        }
      }
      if(count == 7) { // reset flags and counters
        scoreBallArray[0] /* scoreBallActive */ = false;
        allowManualIntakingWhileScoring = false;
        count = 0;
        scoreTwo = false;
        conditionOne = false;
        conditionTwo = false;
        positionArray[3] /* evaluated */ = false;
        opticalLightOff();
      }
    }

    /* true-false. one ball initially in position one */
    else if(positionArray[1] == true && positionArray[2] == false) {
      if(allowManualIntakingWhileScoring == false) { // intakes are not used here, allow manual controls
        allowManualIntakingWhileScoring = true;
      }
      if(count == 0) {
        opticalLightOn();
        count = 1;
      }
      if(count == 1) { // check the color of the ball
        evaluatePositionNoFlags(positionArrayNoFlags);
        identifyColorNoFlags();
        if(pos1Status == 1 /* dispose */) { // if the ball is the color of the opposing alliance
          conveyorSpin(true, 600);
          sorterSpin(false, 600); // dispose
          count = 2;
          wait(2, msec);
        }
        else if(pos1Status == 0 /* score */ || pos1Status == 2 /* unknown */) {
          conveyorSpin(true, 600); // if the ball is the color of my alliance
          sorterScore(); // score
          count = 4;
          wait(2, msec);
        }
      }
      if(count == 2) { // only executes if ball is to be disposed
        if(DistancePos2.objectDistance(mm) < distancePos2Occupied) {
          count = 3; // check that the ball reaches position two
        }
      }
      if(count == 3) {
        if(DistancePos2.objectDistance(mm) >= distancePos2Unoccupied) {
          conveyorStop(); // stop once the ball has been disposed
          sorterStop();
          count = 1; // descore and re-evaluate
          conditionOne = false;
          conditionTwo = false;
          evaluatePosition(positionArray); // will run false-false next iteration
        }
      }
      if(count == 4) { // only executes if ball is to be scored
        if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied) {
          conveyorStop(); // checks that the ball is in the process of being scored
          count = 5;
        }
      }
      if(count == 5) {
        if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
          sorterStop(); // stop once ball is scored
          count = 6;
        }
      }
      if(count == 6) { // reset flags and counters
        scoreBallArray[0] /* scoreBallActive */ = false;
        allowManualIntakingWhileScoring = false;
        count = 0;
        scoreTwo = false;
        conditionOne = false;
        conditionTwo = false;
        positionArray[3] /* evaluated */ = false;
        opticalLightOff();
      }
    }

    /* false-true. one ball initially in position two */
    else if(positionArray[1] == false && positionArray[2] == true) {
      if(allowManualIntakingWhileScoring == false) {
        allowManualIntakingWhileScoring = true; // intakes are not used here, allow manual controls
      }
      if(count == 0) {
        opticalLightOn();
        count = 1;
      }
      if(count == 1) { // check the color of the ball
        evaluatePositionNoFlags(positionArrayNoFlags);
        identifyColorNoFlags();
        if(pos2Status == 1 /* dispose */) { // if the ball is the color of the opposing alliance
          conveyorSpin(true, 600);
          sorterSpin(false, 600); // dispose
          count = 2;
          wait(2, msec);
        }
        else if(pos2Status == 0 /* score */ || pos2Status == 2 /* unknown */) {
          conveyorSpin(true, 600); // if the ball is the color of my alliance
          sorterScore(); // score
          count = 3;
          wait(2, msec);
        }
      }
      if(count == 2) { // only executes if ball is to be disposed
        if(DistancePos2.objectDistance(mm) >= distancePos2Unoccupied) {
          conveyorStop(); // stop once the ball has been disposed
          sorterStop();
          count = 1; // descore and re-evaluate
          conditionOne = false;
          conditionTwo = false;
          evaluatePosition(positionArray); // will run false-false next iteration
        }
      }
      if(count == 3) { // only executes if ball is to be scored
        if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied) {
          conveyorStop(); // check that the ball is in the process of being scored
          count = 4;
        }
      }
      if(count == 4) {
        if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
          sorterStop(); // stop once ball is scored
          count = 5;
        }
      }
      if(count == 5) { // reset flags and counters
        scoreBallArray[0] /* scoreBallActive */ = false;
        allowManualIntakingWhileScoring = false;
        count = 0;
        scoreTwo = false;
        conditionOne = false;
        conditionTwo = false;
        positionArray[3] /* evaluated */ = false;
        opticalLightOff();
      }
    }

    /* true-true. two balls initially in positions one and two */
    else if(positionArray[1] == true && positionArray[2] == true) {
      if(allowManualIntakingWhileScoring == false) {
        allowManualIntakingWhileScoring = true; // intakes are not used here, allow manual controls
      }
      if(count == 0) {
        opticalLightOn();
        count = 1;
      }
      if(count == 1) { // check the color of both balls
        evaluatePositionNoFlags(positionArrayNoFlags);
        identifyColorNoFlags();
        if(pos1Status == 1 /* dispose */ && pos2Status == 1 /* dispose */ ) {
          conveyorSpin(true, 600); // dispose both, both are the color of the opposing alliance
          sorterSpin(false, 600);
          count = 2;
          wait(2, msec);
        }
        else if(pos1Status == 1 /* dispose */ && 
        (pos2Status == 0 /* score */ || pos2Status == 2 /* unknown */)) {
          conveyorSpin(true, 600); // score then dispose, the higher ball is the color of my alliance,
          sorterScore(); // the lower ball is the color of the opposing alliance
          count = 5;
          wait(2, msec);
        }
        else if((pos1Status == 0 /* score */ || pos1Status == 2 /* unknown */) 
        && pos2Status == 1 /* dispose */) {
          conveyorSpin(true, 600); // dispose then score, the higher ball is the color of the opposing alliance,
          sorterSpin(false, 600); // the lower ball is the color of my alliance
          count = 9;
          wait(2, msec);
        }
        else if(scoreTwo == false && (pos1Status == 0 /* score */ || pos1Status == 2 /* unknown */) 
        && (pos2Status == 0 /* score */ || pos2Status == 2 /* unknown */)) {
          conveyorSpin(true, 600); // score then keep, both balls are the color of my alliance and scoreTwo is false
          sorterScore();
          count = 12;
          wait(2, msec);
        }
        else if(scoreTwo == true && (pos1Status == 0 /* score */ || pos1Status == 2 /* unknown */) 
        && (pos2Status == 0 /* score */ || pos2Status == 2 /* unknown */)) {
          conveyorSpin(true, 600); // score two, both balls are the color of my alliance
          sorterScore();
          count = 15;
          wait(2, msec);
        }
      }
      if(count == 2) { // only executes when both balls are to be disposed
        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          count = 3; // check that one ball is disposed and the second is in between positions one and two
        }
      }
      if(count == 3) {
        if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
          count = 4; // check that the second ball reaches position two
        }
      }
      if(count == 4) { // stop once both balls have been disposed
        if(DistancePos2.objectDistance(mm) >= distancePos2Unoccupied) {
          conveyorStop();
          sorterStop();
          count = 1; // descore and re-evaluate
          conditionOne = false;
          conditionTwo = false;
          evaluatePosition(positionArray); // will run false-false next iteration
        }
      }
      if(count == 5) { // only executes for score then dispose
        if(DistancePos1.objectDistance(mm) >= distancePos1Unoccupied) {
          conveyorStop(); // check that the ball to be disposed is in between positions one and two
          conditionOne = true;
        }
        if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied && DistanceScoring.isObjectDetected() == true) {
          conditionTwo = true; // check that the ball to be scored is in the process of being scored
        }
        if(conditionOne == true && conditionTwo == true) {
          count = 6; // don't move on until both of these conditions have been met
        }
      }
      if(count == 6) {
        if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
          sorterStop(); // check that the ball to be scored is scored
          conveyorSpin(true, 600);
          sorterSpin(false, 600); // dispose the remaining ball
          count = 7;
        }
      }
      if(count == 7) {
        if(DistancePos2.objectDistance(mm) < distancePos2Occupied) {
          count = 8; // check that the ball to be disposed reaches position two
        }
      }
      if(count == 8) { 
        if(DistancePos2.objectDistance(mm) >= distancePos2Unoccupied) {
          conveyorStop(); // stop once second ball is disposed
          sorterStop();
          count = 19; // reset and exit
        }
      }
      if(count == 9) { // only executes for dispose then score
        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          sorterStop();
          sorterScore(); // check that the first ball has been disposed, then score the second ball
          count = 10;
        }
      }
      if(count == 10) {
        if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied) {
          conveyorStop(); // check that the ball to be scored is in the process of being scored
          count = 11;
        }
      }
      if(count == 11) {
        if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
          sorterStop(); // stop once second ball is scored
          count = 19; // reset and exit
        }
      }
      if(count == 12) { // only executes for score then keep
        if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied && DistanceScoring.isObjectDetected() == true) {
          count = 13; // check that the first ball is in the process of being score
        }
      }
      if(count == 13) {
        if(DistanceScoring.objectDistance(mm) > distanceScoringBallSeen || DistanceScoring.isObjectDetected() == false) {
          sorterLock(); // bring second ball to position two after the first is scored
          conveyorStop();
          wait(50, msec);
          // check that the second ball is either in position one or two
          if((DistanceScoring.isObjectDetected() == false || DistanceScoring.objectDistance(mm) > distanceScoringBallSeen) 
          || (DistancePos1.objectDistance(mm) <= distancePos1Occupied && DistancePos1.isObjectDetected() == true) 
          || (DistancePos2.objectDistance(mm) <= distancePos2Occupied && DistancePos2.isObjectDetected() == true)) {
            count = 19; // reset and exit
          }
          else {
            sorterSpin(false, 600); // take ball back down to either position one or two
            conveyorSpin(false, 600);
            count = 14;
          }
        }
      }
      if(count == 14) {
        if((DistancePos1.objectDistance(mm) <= distancePos1Occupied && DistancePos1.isObjectDetected() == true) 
          || (DistancePos2.objectDistance(mm) <= distancePos2Occupied && DistancePos2.isObjectDetected() == true)) {
          sorterLock(); // stop once ball is in either position one or two
          conveyorLock(); 
          count = 19; // reset and exit
        }
      }
      if(count == 15) { // only executes for score two
        if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied) {
          count = 16; // check that the first ball is in the process of being scored
        }
      }
      if(count == 16) {
        if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
          count = 17;  // check that the first ball is scored
        }
      }
      if(count == 17) {
        if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied) {
          count = 18; // check that the second ball is in the process of being scored
        }
      }
      if(count == 18) {
        if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
          sorterStop();
          conveyorStop();
          count = 19;  // check that the second ball is scored
        }
      }
      if(count == 19) { // reset flags and counters
        scoreBallArray[0] /* scoreBallActive */ = false;
        allowManualIntakingWhileScoring = false;
        count = 0;
        scoreTwo = false;
        conditionOne = false;
        conditionTwo = false;
        positionArray[3] /* evaluated */ = false;
        opticalLightOff();
      }
    }
  }
  return scoreBallArray[1];
}

int scoreAndDescore(int scoreAndDescoreArray[1]) {
  if(Controller.ButtonL2.pressing() == true && scoreAndDescoreArray[0] == false && scoreTwo == false) {
    /* the condition that scoreTwo == false is so that the scoreAndDescore macro cannot re-evaluate and execute 
    while the scoreBall macro is in progress */

    int numberOfChecks = 0; // reset flags
    scoreAndDescoreArray[0] = true; /* "lock" all other macros and controls that use the intakes, conveyor, and 
    sorter to prevent conflicting motor commands */

    // for a tenth of a second, check to see if both L1 and L2 are being pressed at the same time, otherwise run scoreAndDescore
    if(positionArray[3] /* evaluated */ == false) {
      while(numberOfChecks <= 25) {
        wait(4, msec);
        if(Controller.ButtonL1.pressing() == true) {
          scoreTwo = true; // redirect to scoreBall macro
          scoreAndDescoreArray[0] = false;
          break;
        }
        numberOfChecks += 1;
      }
    }
  }
  if(scoreAndDescoreArray[0] == true) {
    // only evaluate the position on the first iteration. determine which section of code needs to be run
    if(positionArray[3] /* evaluated */ == false) {
      positionOneHalfOccupied = false; // reset flag used in collectBall macro
      evaluatePosition(positionArray);
    }

    /* true-false or false-true or true-true. can only execute if there is at least one ball initially in the conveyor */
    if(positionArray[1] == true || positionArray[2] == true) {
      if(count == 0) {
        intake();
        count = 1;
      }
      if(count == 1) { // begin scoring when the ball to be descored is in the intakes
        if(DistancePos0.objectDistance(mm) < distancePos0Occupied) {
          conveyorSpin(true, 600);
          sorterScore();
          count = 2;
        }
      }
      if(count == 2) {
        if(DistancePos0.objectDistance(mm) >= distancePos0Unoccupied || 
        (positionArray[1] == false && DistancePos1.objectDistance(mm) < distancePos1Unoccupied)) {
          intakesStop(); // stop once the ball to be descored is in the conveyor
        }
        if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied) {
          count = 3; // check that the ball to be scored is in the process of being scored
        }
      }
      if(count == 3) {
        if(DistancePos0.objectDistance(mm) >= distancePos0Unoccupied || 
        (positionArray[1] == false && DistancePos1.objectDistance(mm) < distancePos1Unoccupied)) {
          intakesStop(); // stop once the ball to be descored is in the conveyor
        } // this if statement is in case the ball has not been descored when count == 2
        if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
          sorterLock(); // stop once the ball to be scored has been scored
          conveyorLock();
          intakesStop();
          count = 4;
        }
      }
      if(count == 4) { // make sure the balls left in the conveyor are positioned correctly
        // if there were initially two balls in the conveyor
        if(positionArray[1] == true && positionArray[2] == true) { 
          wait(50, msec);
          if(DistancePos2.objectDistance(mm) <= distancePos2Occupied) {
            count = 6; // if the balls are positioned correctly, reset and exit
          }
          else {
            if(!(DistancePos1.objectDistance(mm) <= distancePos1Occupied)) {
              conveyorSpin(false, 600); // take the lower ball down to position one
            }
            sorterSpin(false, 600); // take the higher ball down to position two
            count = 5;
          }
        }
        // if there was initially only one ball in the conveyor
        else if(positionArray[1] == true || positionArray[2] == true) { 
          if(DistancePos2.objectDistance(mm) <= distancePos2Occupied) {
            count = 6; // if the ball is positioned correctly, reset and exit
          }
          else {
            sorterSpin(false, 600); // move the ball to position two
            conveyorSpin(true, 600);
            count = 5;
          }
        }
      }
      if(count == 5) {
        if(DistancePos2.objectDistance(mm) <= distancePos2Occupied) {
          sorterLock(); // stop once the ball moving to position two reaches that position
          conveyorLock();
          if(positionArray[1] == true && positionArray[2] == true) { // if two balls in the conveyor
            if(DistancePos1.objectDistance(mm) >= distancePos1Unoccupied) {
              positionOneHalfOccupied = true; // make sure the lower ball ended up in the correct position
            }
          }
          count = 6; // reset and exit
        }
      }
      if(count == 6) { // reset flags and counters
        scoreAndDescoreArray[0] /* scoreAndDescoreActive */ = false;
        count = 0;
        positionArray[3] /* evaluated */ = false;
      }
    }
    
    /* false-false. cannot simultaneously score and descore if the conveyor is initially empty */
    else { // reset flags and counters
      scoreAndDescoreArray[0] /* scoreAndDescoreActive */ = false;
      count = 0;
      positionArray[3] /* evaluated */ = false;
      Controller.rumble("."); // alert that the macro cannot execute
    }
  }
  return scoreAndDescoreArray[1];
}

int disposeBall(int disposeBallArray[1]) {
  if(Controller.ButtonX.pressing() == true && disposeBallArray[0] == false) {
    /* "lock" all other macros and controls that use the intakes, conveyor, and sorter to prevent conflicting motor commands */
    disposeBallArray[0] = true;
  }
  if(disposeBallArray[0] == true) {
    // only evaluate the position on the first iteration. determine which section of code needs to be run
    if(positionArray[3] /* evaluated */ == false) {
      positionOneHalfOccupied = false; // reset flag used in collectBall macro
      evaluatePosition(positionArray);
    }

    /* false-false-false. cannot dispose if there are no balls in the robot */
    if(positionArray[0] == false && positionArray[1] == false && positionArray[2] == false) {
      disposeBallArray[0] /* disposeBallActive */ = false;
      positionArray[3] /* evaluated */ = false; // reset flags
    }

    /* true-false-false. one ball in the intakes */
    else if(positionArray[0] == true && positionArray[1] == false && positionArray[2] == false) {

      if(count == 0) {
        intake();
        count = 1;
      }
      if(count == 1) { // start the conveyor once the ball reaches the back of the intakes
        if(DistancePos0.objectDistance(mm) <= distancePos0Occupied) {
          conveyorSpin(true, 600);
          sorterSpin(false, 600); // dispose
          count = 2;
        }
      }
      if(count == 2) { // stop once ball is in the conveyor
        if(DistancePos1.objectDistance(mm) < distancePos1Occupied) {
          intakesStop(); // stop once the ball is out of the intakes
          count = 3;
        }
      }
      if(count == 3) {
        if(DistancePos2.objectDistance(mm) <= distancePos2Unoccupied) {
          count = 4; // check that the ball reaches position two
        }
      }
      if(count == 4) {
        if(DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          conveyorStop(); // stop once ball has been disposed
          sorterStop();
          count = 5; // reset and exit
        }
      }
      if(count == 5) { // reset flags and counters
        disposeBallArray[0] /* disposeBallActive */ = false;
        count = 0;
        positionArray[3] /* evaluated */ = false;
      }
    }

    /* false-true-false or true-true-false. ball to dispose in position one. position two empty */
    else if(positionArray[1] == true && positionArray[2] == false) {
      if(count == 0) {
        conveyorSpin(true, 600); // dispose
        sorterSpin(false, 600);
        count = 1;
      }
      if(count == 1) {
        if(DistancePos2.objectDistance(mm) <= distancePos2Unoccupied) {
          count = 2; // check that the ball to dispose reaches position two
        }
      }
      if(count == 2) {
        if(DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          conveyorStop(); // stop once ball has been disposed
          sorterStop();
          count = 3; // reset and exit
        }
      }
      if(count == 3) { // reset flags and counters
        disposeBallArray[0] /* disposeBallActive */ = false;
        count = 0;
        positionArray[3] /* evaluated */ = false;
      }
    }

    /* false-false-true or true-false-true. ball to dispose in position two. position one empty */
    else if(positionArray[1] == false && positionArray[2] == true) {
      if(count == 0) {
        conveyorSpin(true, 600); // dispose
        sorterSpin(false, 600);
        count = 1;
      }
      if(count == 1) {
        if(DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          conveyorStop(); // stop once ball has been disposed
          sorterStop();
          count = 2; // reset and exit
        }
      }
      if(count == 2) { // reset flags and counters
        disposeBallArray[0] /* disposeBallActive */ = false;
        count = 0;
        positionArray[3] /* evaluated */ = false;
      }
    }

    /* false-true-true or true-true-true. ball to dispose in position two. position one occupied */
    else if(positionArray[1] == true && positionArray[2] == true) {
      if(count == 0) {
        conveyorSpin(true, 600);
        sorterSpin(false, 600); // dispose
        count = 1;
      }
      if(count == 1) {
        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied) {
          sorterLock(); // check that the lower ball is out of position one
          count = 2; // the ball initially in position two must have been disposed in order for this to occur
        }
      }
      if(count == 2) {
        if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
          conveyorStop(); // stop once the ball initially in position one is in position two
          count = 3;
        }
      }
      if(count == 3) { // reset flags and counters
        disposeBallArray[0] /* disposeBallActive */ = false;
        count = 0;
        positionArray[3] /* evaluated */ = false;
      }
    }
  }
  return disposeBallArray[1];
}

/* AUTONOMOUS-ONLY FUNCTIONS */

void unstow() { // spin outward to unstow the intakes
  IntakeMotorLeft.rotateFor(-250,rotationUnits::deg,200,velocityUnits::rpm,false);
  IntakeMotorRight.rotateFor(-250,rotationUnits::deg,200,velocityUnits::rpm,true);
}

void determineHighestEncoderValue() { // check each drivetrain encoder, keep the highest number of degrees
  highestEncoderValue = fabs(DriveMotorLeftFront.rotation(deg));
  if(fabs(DriveMotorLeftBack.rotation(deg)) > highestEncoderValue) {
    highestEncoderValue = fabs(DriveMotorLeftBack.rotation(deg));
  }
  if(fabs(DriveMotorRightFront.rotation(deg)) > highestEncoderValue) {
    highestEncoderValue = fabs(DriveMotorRightFront.rotation(deg));
  }
  if(fabs(DriveMotorRightBack.rotation(deg)) > highestEncoderValue) {
    highestEncoderValue = fabs(DriveMotorRightBack.rotation(deg));
  }
}

int driveForward(int driveArray[6]) { // stepsCompleted, step, speed, degreesToTravel, percentToAccel, percentToDecel
  if(driveArray[1] /* step */ == 0) {
    DriveMotorLeftFront.resetRotation(); // only reset the encoder counts on the first iteration
    DriveMotorLeftBack.resetRotation();
    DriveMotorRightFront.resetRotation();
    DriveMotorRightBack.resetRotation();
    driveArray[1] /* step */ = 1;
  }
  
  // get the current encoder value in degrees and determine how much further
  determineHighestEncoderValue();
  degreesRemaining = driveArray[3] /* degreesToTravel */ - highestEncoderValue;

  /* number of degrees to accelerate / decelerate based on what percentage of the 
  total numbers of degrees to travel I specify as an argument in the driveArray */
  degreesToAccel = (double(driveArray[4]) /* percentToAccel */ / 100) * double(driveArray[3]) /* degreesToTravel */; 
  degreesToDecel = (double(driveArray[5]) /* percentToDecel */ / 100) * double(driveArray[3]) /* degreesToTravel */;

  // percentage of degrees to accelerate / decelerate completed as a fraction
  percentAccelComplete = highestEncoderValue / degreesToAccel;
  percentDecelComplete = (degreesToDecel - degreesRemaining) / degreesToDecel;

  if(highestEncoderValue <= degreesToAccel) { // accelerate if the degreesToAccel has not been completed
    accel = true;
    fullSpeed = false;
    decel = false;
  }
  else if(highestEncoderValue > degreesToAccel 
  && highestEncoderValue < (driveArray[3] /* degreesToTravel */ - degreesToDecel)) {
    accel = false;
    fullSpeed = true; // move at the constant target speed between acceleration and deceleration
    decel = false;
  }
  else if(highestEncoderValue >= (driveArray[3] /* degreesToTravel */ - degreesToDecel)) {
    accel = false;
    fullSpeed = false;
    decel = true; // decelerate if the degreesToDecel has not been completed
  }

  if(accel == true) {
    setSpeed = percentAccelComplete * driveArray[2] /* speed */; // accelerate proportional to distance traveled
  }
  else if(fullSpeed == true) {
    setSpeed = double(driveArray[2] /* speed */); // set to the target speed when not accelerating or decelerating
  }
  else if(decel == true) {
    setSpeed = (1 - percentDecelComplete) * driveArray[2] /* speed */; // decelerate proportional to distance traveled
  }

  if((setSpeed <= 0 && decel == true) || highestEncoderValue >= driveArray[3] /* degreesToTravel */) {
    drivetrainStop();
    driveArray[0] /* stepsCompleted */ = true;
    driveArray[1] /* step */ = 0;
  }
  else {
    if(setSpeed < 50 && accel == true) {
      setSpeed = 50; // baseline speed for acceleration
    }
    if(setSpeed < 10 && decel == true) {
      setSpeed = 10; // lowest speed for deceleration
    }

    /* when accelerating or decelerating, use the inertial sensor's accelerometer for drift correction */
    if(accel == true || decel == true) {
      if(Inertial.acceleration(yaxis) >= -accelerometerDriftThreshold 
      && Inertial.acceleration(yaxis) <= accelerometerDriftThreshold) {
        leftDriftOffset = 0;
        rightDriftOffset = 0; // no correction if drift is within the allowed threshold
      }
      else if(Inertial.acceleration(yaxis) < -accelerometerDriftThreshold) { // if drifting left
        rightDriftOffset = setSpeed / 4; // decrease the right pair by 25% of the set speed to correct to the left
        leftDriftOffset = 0;
      }
      else if(Inertial.acceleration(yaxis) > accelerometerDriftThreshold) { // if drifting right
        leftDriftOffset = setSpeed / 4; // decrease the left pair by 25% of the set speed to correct to the right
        rightDriftOffset = 0;
      }
    }
    /* when moving at a constant speed, use the motor encoders for drift correction
    (accelerometer only works when there is a change in motion) */
    else if(fullSpeed == true) {
      // takes the average to minimize inconsistencies due to wheel slippage
      leftEncoderAverage = (fabs(DriveMotorLeftFront.rotation(deg)) + fabs(DriveMotorRightBack.rotation(deg))) / 2;
      rightEncoderAverage = (fabs(DriveMotorRightFront.rotation(deg)) + fabs(DriveMotorLeftBack.rotation(deg))) / 2;
      
      if(leftEncoderAverage > rightEncoderAverage) { // if drifting right
        // calculate the error by comparing the difference to the larger value
        driftError = (leftEncoderAverage - rightEncoderAverage) / leftEncoderAverage;
        leftDriftOffset = setSpeed * driftError; // correct proportionally to the amount off
        rightDriftOffset = 0;
      }
      else if(leftEncoderAverage < rightEncoderAverage) { // if drifting left
        // calculate the error by comparing the difference to the larger value
        driftError = (rightEncoderAverage - leftEncoderAverage) / rightEncoderAverage;
        rightDriftOffset = setSpeed * driftError; // correct proportionally to the amount off
        leftDriftOffset = 0;
      }
      else {
        leftDriftOffset = 0; // no correction if no drifting
        rightDriftOffset = 0;
      }
    }
    // spin the wheels at the speed determined for accel or fullSpeed or decel with the drift correction factored in
    DriveMotorLeftFront.spin(directionType::fwd,setSpeed - leftDriftOffset,velocityUnits::rpm);
    DriveMotorLeftBack.spin(directionType::fwd,setSpeed - rightDriftOffset,velocityUnits::rpm);
    DriveMotorRightFront.spin(directionType::fwd,setSpeed - rightDriftOffset,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::fwd,setSpeed - leftDriftOffset,velocityUnits::rpm);
  }
  return driveArray[6];
}

int driveBackward(int driveArray[6]) { // stepsCompleted, step, speed, degreesToTravel, percentToAccel, percentToDecel
  if(driveArray[1] /* step */ == 0) {
    DriveMotorLeftFront.resetRotation(); // only reset the encoder counts on the first iteration
    DriveMotorLeftBack.resetRotation();
    DriveMotorRightFront.resetRotation();
    DriveMotorRightBack.resetRotation();
    driveArray[1] /* step */ = 1;
  }
  
  // get the current encoder value in degrees and determine how much further
  determineHighestEncoderValue();
  degreesRemaining = driveArray[3] /* degreesToTravel */ - highestEncoderValue;

  /* number of degrees to accelerate / decelerate based on what percentage of the 
  total numbers of degrees to travel I specify as an argument in the driveArray */
  degreesToAccel = (double(driveArray[4]) /* percentToAccel */ / 100) * double(driveArray[3]) /* degreesToTravel */; 
  degreesToDecel = (double(driveArray[5]) /* percentToDecel */ / 100) * double(driveArray[3]) /* degreesToTravel */;

  // percentage of degrees to accelerate / decelerate completed as a fraction
  percentAccelComplete = highestEncoderValue / degreesToAccel;
  percentDecelComplete = (degreesToDecel - degreesRemaining) / degreesToDecel;

  if(highestEncoderValue <= degreesToAccel) { // accelerate if the degreesToAccel has not been completed
    accel = true;
    fullSpeed = false;
    decel = false;
  }
  else if(highestEncoderValue > degreesToAccel 
  && highestEncoderValue < (driveArray[3] /* degreesToTravel */ - degreesToDecel)) {
    accel = false;
    fullSpeed = true; // move at the constant target speed between acceleration and deceleration
    decel = false;
  }
  else if(highestEncoderValue >= (driveArray[3] /* degreesToTravel */ - degreesToDecel)) {
    accel = false;
    fullSpeed = false;
    decel = true; // decelerate if the degreesToDecel has not been completed
  }

  if(accel == true) {
    setSpeed = percentAccelComplete * driveArray[2] /* speed */; // accelerate proportional to distance traveled
  }
  else if(fullSpeed == true) {
    setSpeed = double(driveArray[2] /* speed */); // set to the target speed when not accelerating or decelerating
  }
  else if(decel == true) {
    setSpeed = (1 - percentDecelComplete) * driveArray[2] /* speed */; // decelerate proportional to distance traveled
  }

  if((setSpeed <= 0 && decel == true) || highestEncoderValue >= driveArray[3] /* degreesToTravel */) {
    drivetrainStop();
    driveArray[0] /* stepsCompleted */ = true;
    driveArray[1] /* step */ = 0;
  }
  else {
    if(setSpeed < 50 && accel == true) {
      setSpeed = 50; // baseline speed for acceleration
    }
    if(setSpeed < 10 && decel == true) {
      setSpeed = 10; // lowest speed for deceleration
    }

    /* when accelerating or decelerating, use the inertial sensor's accelerometer for drift correction */
    if(accel == true || decel == true) {
      if(Inertial.acceleration(yaxis) >= -accelerometerDriftThreshold 
      && Inertial.acceleration(yaxis) <= accelerometerDriftThreshold) {
        leftDriftOffset = 0;
        rightDriftOffset = 0; // no correction if drift is within the allowed threshold
      }
      else if(Inertial.acceleration(yaxis) > accelerometerDriftThreshold) { // if drifting left
        rightDriftOffset = setSpeed / 4; // decrease the right pair by 25% of the set speed to correct to the left
        leftDriftOffset = 0;
      }
      else if(Inertial.acceleration(yaxis) < -accelerometerDriftThreshold) { // if drifting right
        leftDriftOffset = setSpeed / 4; // decrease the left pair by 25% of the set speed to correct to the right
        rightDriftOffset = 0;
      }
    }
    /* when moving at a constant speed, use the motor encoders for drift correction
    (accelerometer only works when there is a change in motion) */
    else if(fullSpeed == true) {
      // takes the average to minimize inconsistencies due to wheel slippage
      leftEncoderAverage = (fabs(DriveMotorLeftFront.rotation(deg)) + fabs(DriveMotorRightBack.rotation(deg))) / 2;
      rightEncoderAverage = (fabs(DriveMotorRightFront.rotation(deg)) + fabs(DriveMotorLeftBack.rotation(deg))) / 2;
    
      if(leftEncoderAverage > rightEncoderAverage) { // if drifting right
        // calculate the error by comparing the difference to the larger value
        driftError = (leftEncoderAverage - rightEncoderAverage) / leftEncoderAverage;
        leftDriftOffset = setSpeed * driftError; // correct proportionally to the amount off
        rightDriftOffset = 0;
      }
      else if(leftEncoderAverage < rightEncoderAverage) { // if drifting left
        // calculate the error by comparing the difference to the larger value
        driftError = (rightEncoderAverage - leftEncoderAverage) / rightEncoderAverage;
        rightDriftOffset = setSpeed * driftError; // correct proportionally to the amount off
        leftDriftOffset = 0;
      }
      else {
        leftDriftOffset = 0; // no correction if no drifting
        rightDriftOffset = 0;
      }
    }
    // spin the wheels at the speed determined for accel or fullSpeed or decel with the drift correction factored in
    DriveMotorLeftFront.spin(directionType::rev,setSpeed - leftDriftOffset,velocityUnits::rpm);
    DriveMotorLeftBack.spin(directionType::rev,setSpeed - rightDriftOffset,velocityUnits::rpm);
    DriveMotorRightFront.spin(directionType::rev,setSpeed - rightDriftOffset,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::rev,setSpeed - leftDriftOffset,velocityUnits::rpm);
  }
  return driveArray[6];
}

int strafeLeft(int driveArray[6]) { // stepsCompleted, step, speed, degreesToTravel, percentToAccel, percentToDecel
  if(driveArray[1] /* step */ == 0) {
    DriveMotorLeftFront.resetRotation(); // only reset the encoder counts on the first iteration
    DriveMotorLeftBack.resetRotation();
    DriveMotorRightFront.resetRotation();
    DriveMotorRightBack.resetRotation();
    driveArray[1] /* step */ = 1;
  }
  
  // get the current encoder value in degrees and determine how much further
  determineHighestEncoderValue();
  degreesRemaining = driveArray[3] /* degreesToTravel */ - highestEncoderValue;

  /* number of degrees to accelerate / decelerate based on what percentage of the 
  total numbers of degrees to travel I specify as an argument in the driveArray */
  degreesToAccel = (double(driveArray[4]) /* percentToAccel */ / 100) * double(driveArray[3]) /* degreesToTravel */; 
  degreesToDecel = (double(driveArray[5]) /* percentToDecel */ / 100) * double(driveArray[3]) /* degreesToTravel */;

  // percentage of degrees to accelerate / decelerate completed as a fraction
  percentAccelComplete = highestEncoderValue / degreesToAccel;
  percentDecelComplete = (degreesToDecel - degreesRemaining) / degreesToDecel;

  if(highestEncoderValue <= degreesToAccel) { // accelerate if the degreesToAccel has not been completed
    accel = true;
    fullSpeed = false;
    decel = false;
  }
  else if(highestEncoderValue > degreesToAccel 
  && highestEncoderValue < (driveArray[3] /* degreesToTravel */ - degreesToDecel)) {
    accel = false;
    fullSpeed = true; // move at the constant target speed between acceleration and deceleration
    decel = false;
  }
  else if(highestEncoderValue >= (driveArray[3] /* degreesToTravel */ - degreesToDecel)) {
    accel = false;
    fullSpeed = false;
    decel = true; // decelerate if the degreesToDecel has not been completed
  }

  if(accel == true) {
    setSpeed = percentAccelComplete * driveArray[2] /* speed */; // accelerate proportional to distance traveled
  }
  else if(fullSpeed == true) {
    setSpeed = double(driveArray[2] /* speed */); // set to the target speed when not accelerating or decelerating
  }
  else if(decel == true) {
    setSpeed = (1 - percentDecelComplete) * driveArray[2] /* speed */; // decelerate proportional to distance traveled
  }

  if((setSpeed <= 0 && decel == true) || highestEncoderValue >= driveArray[3] /* degreesToTravel */) {
    drivetrainStop();
    driveArray[0] /* stepsCompleted */ = true;
    driveArray[1] /* step */ = 0;
  }
  else {
    if(setSpeed < 50 && accel == true) {
      setSpeed = 50; // baseline speed for acceleration
    }
    if(setSpeed < 10 && decel == true) {
      setSpeed = 10; // lowest speed for deceleration
    }

    /* when accelerating or decelerating, use the inertial sensor's accelerometer for drift correction */
    if(accel == true || decel == true) {
      if(Inertial.acceleration(xaxis) >= -accelerometerDriftThreshold 
      && Inertial.acceleration(xaxis) <= accelerometerDriftThreshold) {
        leftDriftOffset = 0;
        rightDriftOffset = 0; // no correction if drift is within the allowed threshold
      }
      else if(Inertial.acceleration(xaxis) < -accelerometerDriftThreshold) { // if drifting left
        rightDriftOffset = setSpeed / 4; // decrease the right pair by 25% of the set speed to correct to the left
        leftDriftOffset = 0;
      }
      else if(Inertial.acceleration(xaxis) > accelerometerDriftThreshold) { // if drifting right
        leftDriftOffset = setSpeed / 4; // decrease the left pair by 25% of the set speed to correct to the right
        rightDriftOffset = 0;
      }
    }
    /* when moving at a constant speed, use the motor encoders for drift correction
    (accelerometer only works when there is a change in motion) */
    else if(fullSpeed == true) {
      // takes the average to minimize inconsistencies due to wheel slippage
      leftEncoderAverage = (fabs(DriveMotorLeftBack.rotation(deg)) + fabs(DriveMotorRightFront.rotation(deg))) / 2;
      rightEncoderAverage = (fabs(DriveMotorRightBack.rotation(deg)) + fabs(DriveMotorLeftFront.rotation(deg))) / 2;
    
      if(leftEncoderAverage > rightEncoderAverage) { // if drifting right
        // calculate the error by comparing the difference to the larger value
        driftError = (leftEncoderAverage - rightEncoderAverage) / leftEncoderAverage;
        leftDriftOffset = setSpeed * driftError; // correct proportionally to the amount off
        rightDriftOffset = 0;
      }
      else if(leftEncoderAverage < rightEncoderAverage) { // if drifting left
        // calculate the error by comparing the difference to the larger value
        driftError = (rightEncoderAverage - leftEncoderAverage) / rightEncoderAverage;
        rightDriftOffset = setSpeed * driftError; // correct proportionally to the amount off
        leftDriftOffset = 0;
      }
      else {
        leftDriftOffset = 0; // no correction if no drifting
        rightDriftOffset = 0;
      }
    }
    // spin the wheels at the speed determined for accel or fullSpeed or decel with the drift correction factored in
    DriveMotorLeftFront.spin(directionType::rev,setSpeed - rightDriftOffset,velocityUnits::rpm);
    DriveMotorLeftBack.spin(directionType::fwd,setSpeed - leftDriftOffset,velocityUnits::rpm);
    DriveMotorRightFront.spin(directionType::fwd,setSpeed - leftDriftOffset,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::rev,setSpeed - rightDriftOffset,velocityUnits::rpm);
  }
  return driveArray[6];
}

int strafeRight(int driveArray[6]) { // stepsCompleted, step, speed, degreesToTravel, percentToAccel, percentToDecel
  if(driveArray[1] /* step */ == 0) {
    DriveMotorLeftFront.resetRotation(); // only reset the encoder counts on the first iteration
    DriveMotorLeftBack.resetRotation();
    DriveMotorRightFront.resetRotation();
    DriveMotorRightBack.resetRotation();
    driveArray[1] /* step */ = 1;
  }
  
  // get the current encoder value in degrees and determine how much further
  determineHighestEncoderValue();
  degreesRemaining = driveArray[3] /* degreesToTravel */ - highestEncoderValue;

  /* number of degrees to accelerate / decelerate based on what percentage of the 
  total numbers of degrees to travel I specify as an argument in the driveArray */
  degreesToAccel = (double(driveArray[4]) /* percentToAccel */ / 100) * double(driveArray[3]) /* degreesToTravel */; 
  degreesToDecel = (double(driveArray[5]) /* percentToDecel */ / 100) * double(driveArray[3]) /* degreesToTravel */;

  // percentage of degrees to accelerate / decelerate completed as a fraction
  percentAccelComplete = highestEncoderValue / degreesToAccel;
  percentDecelComplete = (degreesToDecel - degreesRemaining) / degreesToDecel;

  if(highestEncoderValue <= degreesToAccel) { // accelerate if the degreesToAccel has not been completed
    accel = true;
    fullSpeed = false;
    decel = false;
  }
  else if(highestEncoderValue > degreesToAccel 
  && highestEncoderValue < (driveArray[3] /* degreesToTravel */ - degreesToDecel)) {
    accel = false;
    fullSpeed = true; // move at the constant target speed between acceleration and deceleration
    decel = false;
  }
  else if(highestEncoderValue >= (driveArray[3] /* degreesToTravel */ - degreesToDecel)) {
    accel = false;
    fullSpeed = false;
    decel = true; // decelerate if the degreesToDecel has not been completed
  }

  if(accel == true) {
    setSpeed = percentAccelComplete * driveArray[2] /* speed */; // accelerate proportional to distance traveled
  }
  else if(fullSpeed == true) {
    setSpeed = double(driveArray[2] /* speed */); // set to the target speed when not accelerating or decelerating
  }
  else if(decel == true) {
    setSpeed = (1 - percentDecelComplete) * driveArray[2] /* speed */; // decelerate proportional to distance traveled
  }

  if((setSpeed <= 0 && decel == true) || highestEncoderValue >= driveArray[3] /* degreesToTravel */) {
    drivetrainStop();
    driveArray[0] /* stepsCompleted */ = true;
    driveArray[1] /* step */ = 0;
  }
  else {
    if(setSpeed < 50 && accel == true) {
      setSpeed = 50; // baseline speed for acceleration
    }
    if(setSpeed < 10 && decel == true) {
      setSpeed = 10; // lowest speed for deceleration
    }

    /* when accelerating or decelerating, use the inertial sensor's accelerometer for drift correction */
    if(accel == true || decel == true) {
      if(Inertial.acceleration(xaxis) >= -accelerometerDriftThreshold 
      && Inertial.acceleration(xaxis) <= accelerometerDriftThreshold) {
        leftDriftOffset = 0;
        rightDriftOffset = 0; // no correction if drift is within the allowed threshold
      }
      else if(Inertial.acceleration(xaxis) > accelerometerDriftThreshold) { // if drifting left
        rightDriftOffset = setSpeed / 4; // decrease the right pair by 25% of the set speed to correct to the left
        leftDriftOffset = 0;
      }
      else if(Inertial.acceleration(xaxis) < -accelerometerDriftThreshold) { // if drifting right
        leftDriftOffset = setSpeed / 4; // decrease the left pair by 25% of the set speed to correct to the right
        rightDriftOffset = 0;
      }
    }
    /* when moving at a constant speed, use the motor encoders for drift correction
    (accelerometer only works when there is a change in motion) */
    else if(fullSpeed == true) {
      // takes the average to minimize inconsistencies due to wheel slippage
      leftEncoderAverage = (fabs(DriveMotorLeftBack.rotation(deg)) + fabs(DriveMotorRightFront.rotation(deg))) / 2;
      rightEncoderAverage = (fabs(DriveMotorRightBack.rotation(deg)) + fabs(DriveMotorLeftFront.rotation(deg))) / 2;
    
      if(leftEncoderAverage > rightEncoderAverage) { // if drifting right
        // calculate the error by comparing the difference to the larger value
        driftError = (leftEncoderAverage - rightEncoderAverage) / leftEncoderAverage;
        leftDriftOffset = setSpeed * driftError; // correct proportionally to the amount off
        rightDriftOffset = 0;
      }
      else if(leftEncoderAverage < rightEncoderAverage) { // if drifting left
        // calculate the error by comparing the difference to the larger value
        driftError = (rightEncoderAverage - leftEncoderAverage) / rightEncoderAverage;
        rightDriftOffset = setSpeed * driftError; // correct proportionally to the amount off
        leftDriftOffset = 0;
      }
      else {
        leftDriftOffset = 0; // no correction if no drifting
        rightDriftOffset = 0;
      }
    }
    // spin the wheels at the speed determined for accel or fullSpeed or decel with the drift correction factored in
    DriveMotorLeftFront.spin(directionType::fwd,setSpeed - rightDriftOffset,velocityUnits::rpm);
    DriveMotorLeftBack.spin(directionType::rev,setSpeed - leftDriftOffset,velocityUnits::rpm);
    DriveMotorRightFront.spin(directionType::rev,setSpeed - leftDriftOffset,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::fwd,setSpeed - rightDriftOffset,velocityUnits::rpm);
  }
  return driveArray[6];
}

bool negativeToPositive;
int turnLeftWithInertial(int turnArray[6]) { // stepsCompleted, step, speed, angle, percentToAccel, percentToDecel
  if(turnArray[1] /* step */ == 0) {
    negativeToPositive = false;
    initialAngle = Inertial.yaw();
    if(initialAngle < 0 && turnArray[3] /* angle */ > 0) {
      initialAngle = 180 + (180 + initialAngle);
      negativeToPositive = true;
    }
    totalDegreesToTurn = fabs(double(turnArray[3] /* angle */ - initialAngle));
    turnArray[1] /* step */ = 1;
  }

  // get the current angle and determine how much further
  currentAngle = Inertial.yaw();
  if(negativeToPositive == true && currentAngle < 0) {
    currentAngle = 180 + (180 + currentAngle);
  }
  degreesTurned = fabs(currentAngle - initialAngle);
  degreesRemaining = fabs(turnArray[3] /* angle */ - currentAngle);

  /* number of degrees to accelerate / decelerate based on what percentage of the 
  total numbers of degrees to turn */
  degreesToAccel = (double(turnArray[4]) /* percentToAccel */ / 100) * totalDegreesToTurn; 
  degreesToDecel = (double(turnArray[5]) /* percentToDecel */ / 100) * totalDegreesToTurn;

  // percentage of degrees to accelerate / decelerate completed as a fraction
  percentAccelComplete = fabs((currentAngle - initialAngle)) / degreesToAccel;
  percentDecelComplete = (degreesToDecel - degreesRemaining) / degreesToDecel;

  if(degreesTurned <= degreesToAccel) { // accelerate if the degreesToAccel has not been completed
    accel = true;
    fullSpeed = false;
    decel = false;
  }
  else if(degreesTurned > degreesToAccel && degreesTurned < (totalDegreesToTurn - degreesToDecel)) {
    accel = false;
    fullSpeed = true; // move at the constant target speed between acceleration and deceleration
    decel = false;
  }
  else if(degreesTurned >= (totalDegreesToTurn - degreesToDecel)) {
    accel = false;
    fullSpeed = false;
    decel = true; // decelerate if the degreesToDecel has not been completed
  }

  if(accel == true) {
    setSpeed = percentAccelComplete * turnArray[2] /* speed */; // accelerate proportional to amount left to turn
  }
  else if(fullSpeed == true) {
    setSpeed = double(turnArray[2] /* speed */); // set to the target speed when not accelerating or decelerating
  }
  else if(decel == true) {
    setSpeed = (1 - percentDecelComplete) * turnArray[2] /* speed */; // decelerate proportional to amount left to turn
  }

  if((setSpeed <= 0 && decel == true) || (Inertial.yaw() <= turnArray[3] /* angle */ && negativeToPositive == false)
  || currentAngle <= turnArray[3] /* angle */) {
    drivetrainStop();
    turnArray[0] /* stepsCompleted */ = true;
    turnArray[1] /* step */ = 0;
  }
  else {
    if(setSpeed < 50 && accel == true) {
      setSpeed = 50; // baseline speed for acceleration
    }
    if(setSpeed < 10 && decel == true) {
      setSpeed = 10; // lowest speed for deceleration
    }
    DriveMotorLeftFront.spin(directionType::rev,setSpeed,velocityUnits::rpm);
    DriveMotorLeftBack.spin(directionType::rev,setSpeed,velocityUnits::rpm);
    DriveMotorRightFront.spin(directionType::fwd,setSpeed,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::fwd,setSpeed,velocityUnits::rpm);
  }
  return turnArray[6];
}

bool positiveToNegative;
int turnRightWithInertial(int turnArray[6]) { // stepsCompleted, step, speed, angle, percentToAccel, percentToDecel  
  if(turnArray[1] /* step */ == 0) {
    positiveToNegative = false;
    initialAngle = Inertial.yaw();
    if(initialAngle > 0 && turnArray[3] /* angle */ < 0) {
      initialAngle = -180 - (180 - initialAngle);
      positiveToNegative = true;
    }
    totalDegreesToTurn = fabs(double(turnArray[3] /* angle */ - initialAngle));
    turnArray[1] /* step */ = 1;
  }

  // get the current angle and determine how much further
  currentAngle = Inertial.yaw();
  if(positiveToNegative == true && currentAngle > 0) {
    currentAngle = -180 - (180 - currentAngle);
  }
  degreesTurned = fabs(currentAngle - initialAngle);
  degreesRemaining = fabs(turnArray[3] /* angle */ - currentAngle);

  /* number of degrees to accelerate / decelerate based on what percentage of the 
  total numbers of degrees to turn */
  degreesToAccel = (double(turnArray[4]) /* percentToAccel */ / 100) * totalDegreesToTurn; 
  degreesToDecel = (double(turnArray[5]) /* percentToDecel */ / 100) * totalDegreesToTurn;

  // percentage of degrees to accelerate / decelerate completed as a fraction
  percentAccelComplete = fabs((currentAngle - initialAngle)) / degreesToAccel;
  percentDecelComplete = (degreesToDecel - degreesRemaining) / degreesToDecel;

  if(degreesTurned <= degreesToAccel) { // accelerate if the degreesToAccel has not been completed
    accel = true;
    fullSpeed = false;
    decel = false;
  }
  else if(degreesTurned > degreesToAccel && degreesTurned < (totalDegreesToTurn - degreesToDecel)) {
    accel = false;
    fullSpeed = true; // move at the constant target speed between acceleration and deceleration
    decel = false;
  }
  else if(degreesTurned >= (totalDegreesToTurn - degreesToDecel)) {
    accel = false;
    fullSpeed = false;
    decel = true; // decelerate if the degreesToDecel has not been completed
  }

  if(accel == true) {
    setSpeed = percentAccelComplete * turnArray[2] /* speed */; // accelerate proportional to amount left to turn
  }
  else if(fullSpeed == true) {
    setSpeed = double(turnArray[2] /* speed */); // set to the target speed when not accelerating or decelerating
  }
  else if(decel == true) {
    setSpeed = (1 - percentDecelComplete) * turnArray[2] /* speed */; // decelerate proportional to amount left to turn
  }

  if((setSpeed <= 0 && decel == true) || (Inertial.yaw() >= turnArray[3] /* angle */ && positiveToNegative == false)
  || currentAngle >= turnArray[3] /* angle */) {
    drivetrainStop();
    turnArray[0] /* stepsCompleted */ = true;
    turnArray[1] /* step */ = 0;
  }
  else {
    if(setSpeed < 50 && accel == true) {
      setSpeed = 50; // baseline speed for acceleration
    }
    if(setSpeed < 10 && decel == true) {
      setSpeed = 10; // lowest speed for deceleration
    }
    DriveMotorLeftFront.spin(directionType::fwd,setSpeed,velocityUnits::rpm);
    DriveMotorLeftBack.spin(directionType::fwd,setSpeed,velocityUnits::rpm);
    DriveMotorRightFront.spin(directionType::rev,setSpeed,velocityUnits::rpm);
    DriveMotorRightBack.spin(directionType::rev,setSpeed,velocityUnits::rpm);
  }
  return turnArray[6];
}

void alignWithGoal(int velocity) { // used while scoring or descoring in autonomous
  DriveMotorLeftFront.spin(directionType::fwd,velocity,velocityUnits::rpm);
  DriveMotorLeftBack.spin(directionType::fwd,velocity,velocityUnits::rpm);
  DriveMotorRightFront.spin(directionType::fwd,velocity,velocityUnits::rpm);
  DriveMotorRightBack.spin(directionType::fwd,velocity,velocityUnits::rpm);
}

int collectBallAuton(int collectBallArray[1]) { // modified version of the collectBall macro for autonomous
  // only evaluate the position on the first iteration. determine which section of code needs to be run
  if(positionArray[3] /* evaluated */ == false) {
    evaluatePosition(positionArray);
    wrongColor = false;
  }

  /* false-false-false or true-false-false. only dealing with one ball, not initially in the conveyor */
  if((positionArray[0] == false && positionArray[1] == false && positionArray[2] == false) || 
  (positionArray[0] == true && positionArray[1] == false && positionArray[2] == false)) {

    if(count == 0) {
      intake();
      opticalLightOn();
      count = 1;
    }
    if(count == 1) { // start the conveyor once the ball reaches position one
      if(DistancePos0.objectDistance(mm) <= distancePos0Occupied) {
        conveyorSpin(true, 600);
        count = 2;
      }
    }
    if(count == 2) { // stop the intakes once the ball is in the conveyor
      if(DistancePos1.objectDistance(mm) < distancePos1Occupied) {
        intakesStop();
        count = 3;
      }
    }
    if(count == 3) { // stop the conveyor once the ball reaches position two
      if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
        conveyorStop();
        count = 4;
      }
    }
    if(count == 4) { // check positioning and color of ball
      evaluatePositionNoFlags(positionArrayNoFlags);
      identifyColorNoFlags();
      if(pos2Status == 1 /* dispose */) { // if the ball is the color of the opposing alliance
        wrongColor = true; // flag to signal that the ball needs to be disposed at the appropriate point in the routine
        count = 5;
      }
      else if(pos2Status == 0 /* keep */ || pos2Status == 2 /* unknown */) {
        count = 5; // if the ball is the color of my alliance or was unable to be identified
      }
    }
    if(count == 5) { // reset flags and counters
      collectBallArray[0] /* stepsCompleted */ = true;
      count = 0;
      positionArray[3] /* evaluated */ = false;
      opticalLightOff();
    }
  }

  /* false-true-false or true-true-false. dealing with two balls, one initially in position one */
  else if((positionArray[0] == false && positionArray[1] == true && positionArray[2] == false) || 
  (positionArray[0] == true && positionArray[1] == true && positionArray[2] == false)) {

    if(count == 0) {
      intake();
      opticalLightOn();
      count = 1;
    }
    if(count == 1) { // once the second ball is in the intakes, start the conveyor to bring both balls up
      if(DistancePos0.objectDistance(mm) <= distancePos0EnteringConveyor) {
        conveyorSpin(true, 500);
        sorterLock();
        count = 2;
      }
    }
    if(count == 2) { // stop once the higher ball reaches position two
      if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
        intakesStop();
        conveyorStop();
        count = 3;
      }
    }
    if(count == 3) { // check ball positioning and color
      evaluatePositionNoFlags(positionArrayNoFlags);
      identifyColorNoFlags();
      if(pos2Status == 1 /* dispose */) { // if the ball is the color of the opposing alliance
        wrongColor = true; // flag to signal that the ball needs to be disposed at the appropriate point in the routine
        count = 4;
      }
      else if(pos2Status == 0 /* keep */ || pos2Status == 2 /* unknown */) {
        count = 4; // if the ball is the color of my alliance or was unable to be identified
      }
    }
    if(count == 4) { // check that the ball that is supposed to be in position one is positioned correctly 
      if(DistancePos1.objectDistance(mm) >= distancePos1Unoccupied) { 
        positionOneHalfOccupied = true;
      }
      else {
        positionOneHalfOccupied = false;
      }
      // reset flags and counters
      collectBallArray[0] /* stepsCompleted */ = true;
      count = 0;
      positionArray[3] /* evaluated */ = false;
      opticalLightOff();
    }
  }

  /* false-false-true or true-false-true. dealing with two balls, one initially in position two */
  else if((positionArray[0] == false && positionArray[1] == false && positionArray[2] == true) || 
  (positionArray[0] == true && positionArray[1] == false && positionArray[2] == true)) {

    if(count == 0) {
      intake();
      conveyorSpin(false, 600); // bring ball in position two down to position one as the second ball being intaked
      opticalLightOn();
      count = 1;
    }
    if(count == 1) { // stop the conveyor once the ball from position two reaches position one
      if(DistancePos1.objectDistance(mm) < distancePos1Unoccupied) {
        conveyorLock();
        sorterLock();
      }
      if(DistancePos0.objectDistance(mm) <= distancePos0EnteringConveyor && DistancePos0.isObjectDetected() == true) {
        count = 2; // will run true-true-false code next iteration, once the second ball reaches the intakes
        positionArray[1] = true;
        positionArray[2] = false;
        conveyorStop();
        sorterLock();
        intake(); // start to bring both balls up to positions one and two
        conveyorSpin(true, 500);
      }
    }
  }

  /* false-true-true or true-true-true. dealing with three balls, two initially in positions one and two */
  else if((positionArray[0] == false && positionArray[1] == true && positionArray[2] == true) || 
  (positionArray[0] == true && positionArray[1] == true && positionArray[2] == true)) {

    if(count == 0) {
      intake();
      opticalLightOn();
      count = 1;
    }
    if(count == 1) { // stop the intakes once the third ball reaches the intakes
      if(DistancePos0.objectDistance(mm) <= distancePos0Occupied) {
        intakesStop();
        count = 2;
      }
    }
    if(count == 2) { // check the color of the highest ball (in position two)
      identifyColor();
      if(pos2Status == 1 /* dispose */) { // if the ball is the color of the opposing alliance
        wrongColor = true; // flag to signal that the ball needs to be disposed at the appropriate point in the routine
        count = 3;
      }
      else if(pos2Status == 0 /* keep */ || pos2Status == 2 /* unknown */) {
        count = 3; // if the ball is the color of my alliance or was unable to be identified
      }
    }
    if(count == 3) { // reset flags and counters
      collectBallArray[0] /* stepsCompleted */ = true;
      count = 0;
      positionArray[3] /* evaluated */ = false;
      opticalLightOff();
    }
  }
  return collectBallArray[1];
}

int collectBallAutonNoColorID(int collectBallArray[1]) { // modified version of collectBallAuton, does not check colors
  // only evaluate the position on the first iteration. determine which section of code needs to be run
  if(positionArray[3] /* evaluated */ == false) {
    evaluatePosition(positionArray);
  }

  /* false-false-false or true-false-false. only dealing with one ball, not initially in the conveyor */
  if((positionArray[0] == false && positionArray[1] == false && positionArray[2] == false) || 
  (positionArray[0] == true && positionArray[1] == false && positionArray[2] == false)) {

    if(count == 0) {
      intake();
      count = 1;
    }
    if(count == 1) { // start the conveyor once the ball reaches the intakes
      if(DistancePos0.objectDistance(mm) <= distancePos0Occupied) {
        conveyorSpin(true, 600);
        count = 2;
      }
    }
    if(count == 2) { 
      if(DistancePos1.objectDistance(mm) < distancePos1Occupied) {
        intakesStop(); // stop the intakes once the ball is in the conveyor
      }
      if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
        intakesStop(); // stop once the ball reaches position two, and reset the flags and counters
        conveyorStop(); 
        collectBallArray[0] /* stepsCompleted */ = true;
        count = 0;
        positionArray[3] /* evaluated */ = false;
      }
    }
  }

  /* false-true-false or true-true-false. dealing with two balls, one initially in position one */
  else if((positionArray[0] == false && positionArray[1] == true && positionArray[2] == false) || 
  (positionArray[0] == true && positionArray[1] == true && positionArray[2] == false)) {

    if(count == 0) {
      intake();
      count = 1;
    }
    if(count == 1) { // start the conveyor once the second ball is in the intakes
      if(DistancePos0.objectDistance(mm) <= distancePos0EnteringConveyor) {
        conveyorSpin(true, 500);
        sorterLock();
        count = 2;
      }
    }
    if(count == 2) { // stop once the higher ball reaches position two
      if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
        intakesStop();
        conveyorStop();
        count = 3;
      }
    }
    if(count == 3) { // check that the ball that is supposed to be in position one is positioned correctly 
      if(DistancePos1.objectDistance(mm) >= distancePos1Unoccupied) {
        positionOneHalfOccupied = true;
      }
      else {
        positionOneHalfOccupied = false;
      } // reset flags and counters
      collectBallArray[0] /* stepsCompleted */ = true;
      count = 0;
      positionArray[3] /* evaluated */ = false;
    }
  }

  /* false-false-true or true-false-true. dealing with two balls, one initially in position two */
  else if((positionArray[0] == false && positionArray[1] == false && positionArray[2] == true) || 
  (positionArray[0] == true && positionArray[1] == false && positionArray[2] == true)) {

    if(count == 0) {
      intake(); // bring ball in position two down to position one as the second ball in being intaked
      conveyorSpin(false, 600); 
      count = 1;
    }
    if(count == 1) { // stop the conveyor once the ball from position two reaches position one
      if(DistancePos1.objectDistance(mm) < distancePos1Unoccupied) {
        conveyorLock();
        sorterLock();
      }
      if(DistancePos0.objectDistance(mm) <= distancePos0EnteringConveyor && DistancePos0.isObjectDetected() == true) {
        count = 2; // will run true-true-false code next iteration once the second ball reaches the intakes
        positionArray[1] = true;
        positionArray[2] = false;
        conveyorStop();
        sorterLock();
        intake(); // start to bring both balls up to positions one and two
        conveyorSpin(true, 500);
      }
    }
  }

  /* false-true-true or true-true-true. dealing with three balls, two initially in positions one and two */
  else if((positionArray[0] == false && positionArray[1] == true && positionArray[2] == true) || 
  (positionArray[0] == true && positionArray[1] == true && positionArray[2] == true)) {

    if(count == 0) {
      intake();
      count = 1;
    }
    if(count == 1) { // stop the intakes once the third ball reaches the intakes
      if(DistancePos0.objectDistance(mm) <= distancePos0Occupied) {
        intakesStop();
        collectBallArray[0] /* stepsCompleted */ = true; // reset the flags and counters
        count = 0;
        positionArray[3] /* evaluated */ = false;
      }
    }
  }
  return collectBallArray[1];
}

int scoreBallAuton(int scoreBallArray[1]) { // modified version of the scoreBall macro for autonomous
  // only evaluate the position on the first iteration. determine which section of code needs to be run
  if(positionArray[3] /* evaluated */ == false) {
    evaluatePosition(positionArray);
    wrongColor = false;
  }

  /* false-false. initially no balls in the conveyor */
  if(positionArray[1] == false && positionArray[2] == false) {
    if(count == 0) {
      opticalLightOn();
      intake();
      sorterScore();
      conveyorSpin(true, 600);
      count = 1;
    }
    if(count == 1) { // stop once ball is in the conveyor and reaches position two
      if(DistancePos1.objectDistance(mm) <= distancePos1Unoccupied) {
        intakesStop();
      }
      if(DistancePos2.objectDistance(mm) < distancePos2Unoccupied) {
        intakesStop();
        conveyorLock();
        count = 2;
      }
    }
    if(count == 2) { // check positioning and color of ball
      evaluatePositionNoFlags(positionArrayNoFlags);
      identifyColorNoFlags();
      if(pos2Status == 1 /* dispose */) { // if the ball is the color of the opposing alliance
        wrongColor = true; // flag to signal that the ball should not be scored
        count = 5; // reset and exit
      }
      else if(pos2Status == 0 /* score */ || pos2Status == 2 /* unknown */) {
        sorterScore(); // if the ball is the color of my alliance or was unable to be identified
        conveyorSpin(true, 600);
        count = 3;
        wait(2, msec);
      }
    }
    if(count == 3) { // only executes if ball is to be scored
      if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied) {
        count = 4; // check the ball is in the process of being scored
      }
    }
    if(count == 4) {
      if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
        sorterStop(); // stop once the ball has been scored
        conveyorStop();
        count = 5;
      }
    }
    if(count == 5) { // reset flags and counters
      scoreBallArray[0] /* stepsCompleted */ = true;
      count = 0;
      positionArray[3] /* evaluated */ = false;
      opticalLightOff();
    }
  }

  /* true-false. one ball initially in position one */
  else if(positionArray[1] == true && positionArray[2] == false) {
    if(count == 0) { // check color and positioning of ball
      opticalLightOn();
      evaluatePositionNoFlags(positionArrayNoFlags);
      identifyColorNoFlags();
      if(pos1Status == 1 /* dispose */) { // if the ball is the color of the opposing alliance
        wrongColor = true; // flag to signal that the ball should not be scored
        count = 3; // reset and exit
      }
      else if(pos1Status == 0 /* score */ || pos1Status == 2 /* unknown */) {
        conveyorSpin(true, 600); 
        sorterScore(); // if the ball is the color of my alliance or was unable to be identified
        count = 1;
        wait(2, msec);
      }
    }
    if(count == 1) { // only executes if ball is to be scored
      if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied) {
        conveyorStop(); // check that the ball is in the process of being scored
        count = 2;
      }
    }
    if(count == 2) {
      if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
        sorterStop(); // stop once the ball has been scored
        count = 3;
      }
    }
    if(count == 3) { // reset flags and counters
      scoreBallArray[0] /* stepsCompleted */ = true;
      count = 0;
      positionArray[3] /* evaluated */ = false;
      opticalLightOff();
    }
  }

  /* false-true. one ball initially in position two */
  else if(positionArray[1] == false && positionArray[2] == true) {
    if(count == 0) { // check color and positioning of ball
      opticalLightOn();
      evaluatePositionNoFlags(positionArrayNoFlags);
      identifyColorNoFlags();
      if(pos2Status == 1 /* dispose */) { // if the ball is the color of the opposing alliance
        wrongColor = true; // flag to signal that the ball should not be scored
        count = 3; // reset and exit
      }
      else if(pos2Status == 0 /* score */ || pos2Status == 2 /* unknown */) {
        conveyorSpin(true, 600);
        sorterScore(); // if the ball is the color of my alliance or was unable to be identified
        count = 1;
        wait(2, msec);
      }
    }
    if(count == 1) { // only executes if ball is to be scored
      if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied) {
        conveyorStop(); // check that the ball is in the process of being scored
        count = 2;
      }
    }
    if(count == 2) {
      if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
        sorterStop(); // stop once the ball has been scored
        count = 3;
      }
    }
    if(count == 3) { // reset flags and counters
      scoreBallArray[0] /* stepsCompleted */ = true;
      count = 0;
      positionArray[3] /* evaluated */ = false;
      opticalLightOff();
    }
  }

  /* true-true. two balls initially in the conveyor, in positions one and two */
  else if(positionArray[1] == true && positionArray[2] == true) {
    if(count == 0) { // check color and positioning of the ball in position two
      opticalLightOn();
      evaluatePositionNoFlags(positionArrayNoFlags);
      identifyColorNoFlags();
      if(pos2Status == 0 /* score */ || pos2Status == 2 /* unknown */) { 
        conveyorSpin(true, 600); // score then keep
        sorterScore();
        count = 1; // if the ball in position two is the color of my alliance or was unable to be identified
        wait(2, msec);
      }
      else { // if the ball in position two is the color of the opposing alliance
        wrongColor = true; // flag to signal that the ball in position two should not be scored
        count = 4; // reset and exit
      }
    }
    if(count == 1) { // only executes for score then keep
      if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied && DistanceScoring.isObjectDetected() == true) {
        count = 2; // check that the ball in position two is in the process of being scored
      }
    }
    if(count == 2) {
      if(DistanceScoring.objectDistance(mm) > distanceScoringBallSeen || DistanceScoring.isObjectDetected() == false) {
        sorterLock(); // check that the ball from position two has been scored
        conveyorStop();
        wait(50, msec);  // make sure the ball from position one is in either position one or two
        if((DistanceScoring.isObjectDetected() == false || DistanceScoring.objectDistance(mm) > distanceScoringBallSeen) 
        || (DistancePos1.objectDistance(mm) <= distancePos1Occupied && DistancePos1.isObjectDetected() == true) 
        || (DistancePos2.objectDistance(mm) <= distancePos2Occupied && DistancePos2.isObjectDetected() == true)) {
          count = 4; // reset and exit
        }
        else { // if the ball from position one is not in either position one or two, take it down to the nearest position
          sorterSpin(false, 600);
          conveyorSpin(false, 600);
          count = 3;
        }
      }
    }
    if(count == 3) { // stop once the ball reaches either position one or two
      if((DistancePos1.objectDistance(mm) <= distancePos1Occupied && DistancePos1.isObjectDetected() == true) 
        || (DistancePos2.objectDistance(mm) <= distancePos2Occupied && DistancePos2.isObjectDetected() == true)) {
        sorterLock();
        conveyorLock();
        count = 4; // reset and exit
      }
    }
    if(count == 4) { // reset flags and counters
      scoreBallArray[0] /* stepsCompleted */ = true;
      count = 0;
      positionArray[3] /* evaluated */ = false;
      opticalLightOff();
    }
  }
  return scoreBallArray[1];
}

int scoreBallAutonNoColorID(int scoreBallArray[1]) { // modified version of the scoreBallAuton, does not check colors
  // only evaluate the position on the first iteration. determine which section of code needs to be run
  if(positionArray[3] /* evaluated */ == false) {
    evaluatePosition(positionArray);
  }

  /* false-false. initially no balls in the conveyor */
  if(positionArray[1] == false && positionArray[2] == false) {
    if(count == 0) {
      intake();
      sorterScore();
      conveyorSpin(true, 600);
      count = 1;
    }
    if(count == 1) { // stop the intakes once a ball reaches position one
      if(DistancePos1.objectDistance(mm) <= distancePos1Unoccupied) {
        intakesStop();
        count = 2;
      }
    }
    if(count == 2) {
      if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied) {
        count = 3; // check that the ball is in the process of being scored
      }
    }
    if(count == 3) {
      if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
        sorterStop(); // stop once the ball has been scored
        conveyorStop();
        scoreBallArray[0] /* stepsCompleted */ = true; // reset the flags and counters
        count = 0;
        positionArray[3] /* evaluated */ = false;
      }
    }
  }

  /* true-true. two balls initially in the conveyor, in positions one and two */
  else if(positionArray[1] == true && positionArray[2] == true) {
    if(count == 0) {
      conveyorSpin(true, 600); // score the ball in position two, then keep the ball in position one
      sorterScore();
      count = 1;
    }
    if(count == 1) {
      if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied && DistanceScoring.isObjectDetected() == true) {
        count = 2; // check that the ball from position two is in the process of being scored
      }
    }
    if(count == 2) {
      if(DistanceScoring.objectDistance(mm) > distanceScoringBallSeen || DistanceScoring.isObjectDetected() == false) {
        sorterLock(); // check that the ball from position two has been scored
        conveyorStop();
        wait(50, msec); // make sure the ball from position one is in either position one or two
        if((DistanceScoring.isObjectDetected() == false || DistanceScoring.objectDistance(mm) > distanceScoringBallSeen) 
        || (DistancePos1.objectDistance(mm) <= distancePos1Occupied && DistancePos1.isObjectDetected() == true) 
        || (DistancePos2.objectDistance(mm) <= distancePos2Occupied && DistancePos2.isObjectDetected() == true)) {
          count = 4; // reset and exit
        }
        else { // if the ball from position one is not in either position one or two, take it down to the nearest position
          sorterSpin(false, 600);
          conveyorSpin(false, 600);
          count = 3;
        }
      }
    }
    if(count == 3) { // stop once the ball reaches either position one or two
      if((DistancePos1.objectDistance(mm) <= distancePos1Occupied && DistancePos1.isObjectDetected() == true) 
        || (DistancePos2.objectDistance(mm) <= distancePos2Occupied && DistancePos2.isObjectDetected() == true)) {
        sorterLock();
        conveyorLock();
        count = 4; // reset and exit
      }
    }
    if(count == 4) { // reset flags and counters
      scoreBallArray[0] /* stepsCompleted */ = true;
      count = 0;
      positionArray[3] /* evaluated */ = false;
    }
  }

  /* true-false or false-true. one ball initially in either position one or two */
  else if(positionArray[1] == true || positionArray[2] == true) {
    if(count == 0) { // score ball
      conveyorSpin(true, 600);
      sorterScore();
      count = 1;
    }
    if(count == 1) {
      if(DistanceScoring.objectDistance(mm) <= distanceScoringOccupied) {
        conveyorStop(); // check that the ball is in the process of being scored
        count = 2;
      }
    }
    if(count == 2) {
      if(DistanceScoring.objectDistance(mm) > distanceScoringOccupied) {
        sorterStop(); // stop once the ball has been scored
        scoreBallArray[0] /* stepsCompleted */ = true;
        count = 0; // reset flags and counters
        positionArray[3] /* evaluated */ = false;
      }
    }
  }
  return scoreBallArray[1];
}

/* COMPETITION TEMPLATE */

void pre_auton( void ) {
  vexcodeInit(); // Initializing Robot Configuration. DO NOT REMOVE!

  // flags and constants for brain screen autonomous selector
  bool ready = false;
  bool scoreSelected = false;
  bool bothSelected = false;
  int width = 170; // in pixels
  int height = 75; // in pixels

  /* rectangular button coordinates: left x, right x, top y, bottom y */
  int scoreCoordinatesArray[4] = {50, 50 + width, 25, 25 + height};
  int bothCoordinatesArray[4] = {260, 260 + width, 25, 25 + height};
  int readyCoordinatesArray[4] = {155, 155 + width, 125, 125 + height};

  // initial brain screen display
  Brain.Screen.setFont(propXL);
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.setPenColor(cyan);
  Brain.Screen.setPenWidth(2);
  Brain.Screen.drawRectangle(scoreCoordinatesArray[0], scoreCoordinatesArray[2], width, height); // Score
  Brain.Screen.drawRectangle(bothCoordinatesArray[0], bothCoordinatesArray[2], width, height); // Both
  Brain.Screen.printAt(79, 75, false, "SCORE");
  Brain.Screen.printAt(297, 75, false, "BOTH");
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawRectangle(readyCoordinatesArray[0], readyCoordinatesArray[2], width, height); // Ready
  Brain.Screen.printAt(185, 175, false, "READY");

  while(ready == false) { // continues to run until "ready" is selected
    // toggle Score on
    if(Brain.Screen.xPosition() > scoreCoordinatesArray[0] && Brain.Screen.xPosition() < scoreCoordinatesArray[1]
    && Brain.Screen.yPosition() > scoreCoordinatesArray[2] && Brain.Screen.yPosition() < scoreCoordinatesArray[3]
    && scoreSelected == false) {
      score = true; // set flags
      descore = false;
      scoreSelected = true;
      bothSelected = false;

      Brain.Screen.clearLine(1, transparent); // only clear the top row of buttons so "ready" doesn't have to be reprinted
      Brain.Screen.clearLine(2, transparent);
      Brain.Screen.clearLine(3, transparent);

      // brain screen display when "score" is selected
      Brain.Screen.setFillColor(transparent);
      Brain.Screen.setPenColor(cyan);
      Brain.Screen.drawRectangle(bothCoordinatesArray[0], bothCoordinatesArray[2], width, height); // Both
      Brain.Screen.printAt(297, 75, false, "BOTH");
      Brain.Screen.setFillColor(cyan);
      Brain.Screen.setPenColor(transparent);
      Brain.Screen.drawRectangle(scoreCoordinatesArray[0], scoreCoordinatesArray[2], width, height); // Score
      Brain.Screen.printAt(79, 75, false, "SCORE");
    }

    // toggle Both on
    else if(Brain.Screen.xPosition() > bothCoordinatesArray[0] && Brain.Screen.xPosition() < bothCoordinatesArray[1]
    && Brain.Screen.yPosition() > bothCoordinatesArray[2] && Brain.Screen.yPosition() < bothCoordinatesArray[3]
    && bothSelected == false) {
      score = true;
      descore = true; // set flags
      scoreSelected = false;
      bothSelected = true;

      Brain.Screen.clearLine(1, transparent); // only clear the top row of buttons so "ready" doesn't have to be reprinted
      Brain.Screen.clearLine(2, transparent);
      Brain.Screen.clearLine(3, transparent);

      // brain screen display when "both" is selected
      Brain.Screen.setFillColor(transparent);
      Brain.Screen.setPenColor(cyan);
      Brain.Screen.drawRectangle(scoreCoordinatesArray[0], scoreCoordinatesArray[2], width, height); // Score
      Brain.Screen.printAt(79, 75, false, "SCORE");
      Brain.Screen.setFillColor(cyan);
      Brain.Screen.setPenColor(transparent);
      Brain.Screen.drawRectangle(bothCoordinatesArray[0], bothCoordinatesArray[2], width, height); // Both
      Brain.Screen.printAt(297, 75, false, "BOTH");
    }

    // toggle flag for calibration to begin
    else if(Brain.Screen.xPosition() > readyCoordinatesArray[0] && Brain.Screen.xPosition() < readyCoordinatesArray[1]
    && Brain.Screen.yPosition() > readyCoordinatesArray[2] && Brain.Screen.yPosition() < readyCoordinatesArray[3]
    && (scoreSelected == true || bothSelected == true)) {
      ready = true; // set flag

      Brain.Screen.clearLine(4, transparent); // only clear the bottom row of buttons so "score" and "both" don't have to be reprinted
      Brain.Screen.clearLine(5, transparent);
      Brain.Screen.clearLine(6, transparent);

      // brain screen display when "ready" is selected
      Brain.Screen.setFillColor(green);
      Brain.Screen.setPenColor(transparent);
      Brain.Screen.drawRectangle(readyCoordinatesArray[0], readyCoordinatesArray[2], width, height); // Ready
      Brain.Screen.printAt(185, 175, false, "READY");
    }
  }
  
  if(ready == true) { // only executes once "ready" has been selected
    wait(1500, msec); // allows time to put brain cover on without any jolts interfering with inertial sensor calibration

    /* resets all motor encoders to 0 */
    DriveMotorLeftFront.resetRotation(); 
    DriveMotorLeftBack.resetRotation();
    DriveMotorRightFront.resetRotation();
    DriveMotorRightBack.resetRotation();
    ConveyorMotor.resetRotation();
    SortingMotor.resetRotation();
    IntakeMotorLeft.resetRotation();
    IntakeMotorRight.resetRotation();

    /* calibrate inertial sensor and print message to controller once calibrated */
    Inertial.calibrate(); 
    while(true) {
      if(Inertial.isCalibrating() == false) {
        Controller.Screen.setCursor(4, 1);
        Controller.Screen.print("ALL SYSTEMS GO"); // print that the inertial sensor has finished calibrating
        break;
      }
    }

    /* determine color of preload */
    OpticalPos1.setLightPower(25, percent);
    OpticalPos1.setLight(ledState::on);
    wait(1000, msec);
    autoSort = true;
    if(OpticalPos1.color() == red) {
      redAlliance = true;
      Controller.Screen.setCursor(4, 1);
      Controller.Screen.print("RED           "); // print that auto-sorting is set to red
    }
    else if(OpticalPos1.color() == blue) {
      redAlliance = false;
      Controller.Screen.setCursor(4, 1);
      Controller.Screen.print("BLUE          "); // print that auto-sorting is set to blue
    }
    else {
      autoSort = false;
      Controller.Screen.setCursor(4, 1);
      Controller.Screen.print("OFF           "); // print that auto-sorting is off
    }
    OpticalPos1.setLight(ledState::off);
  }
}

void autonomous( void ) {
  /* Initializations for local variables and arrays */
  bool stepsCompleted = false;
  double startTime = 0, startingAngle;
  int step = 0, angle = 0, speed = 0, degreesToTravel = 0, percentToAccel = 0, percentToDecel = 0;
  int driveArray[6] = {stepsCompleted, step, speed, degreesToTravel, percentToAccel, percentToDecel};
  int turnArray[6] = {stepsCompleted, step, speed, angle, percentToAccel, percentToDecel};
  int collectBallArray[1] = {stepsCompleted};
  int scoreBallArray[1] = {stepsCompleted};

  Brain.Timer.reset();
  while(Brain.Timer.value() <= 60) {
    // unstow
    if(routineIndex == 0) {
      unstow();
      intake();
      routineIndex = 1;
    }
    
    // pick up red ball and drive forward
    else if(routineIndex == 1) { // reset flags
      collectBallArray[0] /* stepsCompleted */ = false;
      driveArray[0] /* stepsCompleted */ = false;
      routineIndex = 2;
    }
    else if(routineIndex == 2) { // collecting and driving occurs simultaneously
      if(collectBallArray[0] /* stepsCompleted */ == false) {
        collectBallAutonNoColorID(collectBallArray);
      }
      if(driveArray[0] /* stepsCompleted */ == false) {
        driveArray[2] /* speed */ = 200;
        driveArray[3] /* degreesToTravel */ = 525;
        driveArray[4] /* percentToAccel */ = 10;
        driveArray[5] /* percentToDecel */ = 10;
        driveForward(driveArray);
      }

      if(collectBallArray[0] /* stepsCompleted */ == true && driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 3; // only continues once both actions have been completed
        wait(5, msec);
      }
    }

    // turn toward corner goal on home row
    else if(routineIndex == 3) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 200;
      turnArray[3] /* angle */ = 120;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 20;
      turnRightWithInertial(turnArray);

      if(turnArray[0] /* stepsCompleted */ == true) {
        routineIndex = 4;
        wait(100, msec);
        intake();
      }
    }

    // drive forward into corner goal on home row
    else if(routineIndex == 4) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = 600;
      driveArray[4] /* percentToAccel */ = 10;
      driveArray[5] /* percentToDecel */ = 1;
      driveForward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 5;
        alignWithGoal(100); // a brief correction to make sure the robot has made it to the goal
        wait(300, msec);
        intakesStop();
        wait(200, msec);
      }
    }

    // score one red ball in corner goal on home row
    else if(routineIndex == 5) {
      scoreBallArray[0] /* stepsCompleted */ = false;
      scoreBallAutonNoColorID(scoreBallArray);

      if(scoreBallArray[0] /* stepsCompleted */ == true) {
        if(descore == true) { // based on the brain screen autonomous selection
          routineIndex = 6;
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          routineIndex = 7;
          drivetrainStop();
          startTime = Brain.Timer.value();
          sorterSpin(false, 600);
          conveyorSpin(false, 600);
          waitUntil(DistancePos1.objectDistance(mm) < distancePos1Unoccupied || Brain.Timer.value() >= startTime + 1);
          conveyorLock();
          sorterStop();
          trigCalcCornerToCenter(56, 15);
          outtake(200);
        }
      }
    }

    // descore one blue ball from corner goal on home row. only executes if descore is true
    else if(routineIndex == 6) {
      collectBallArray[0] /* stepsCompleted */ = false;
      collectBallAutonNoColorID(collectBallArray);

      if(collectBallArray[0] /* stepsCompleted */ == true) {
        routineIndex = 7;
        intake();
        waitUntil(DistancePos0.objectDistance(mm) < distancePos0Unoccupied);
        drivetrainStop();
        intakesStop();
        trigCalcCornerToCenter(56, 15);
        intake();
      }
    }

    // back up from corner goal on home row
    else if(routineIndex == 7) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = degreesToReverse;
      driveArray[4] /* percentToAccel */ = 15;
      driveArray[5] /* percentToDecel */ = 15;
      driveBackward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 8;
        intakesStop();
      }
    }

    // turn to be perpendicular to field perimeter
    else if(routineIndex == 8) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 105;
      turnArray[3] /* angle */ = 175;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 20;
      turnRightWithInertial(turnArray);

      if(turnArray[0] /* stepsCompleted */ == true) {
        if(descore == true) { // based on the brain screen autonomous selection
          routineIndex = 9;
          outtake(200); // first blue ball leaving the robot
          waitUntil(DistancePos0.objectDistance(mm) > distancePos0Occupied);
          conveyorSpin(true, 600);
          sorterSpin(true, 325);
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          routineIndex = 11;
          wait(100, msec);
        }
      }
    }

    // outtake two blue balls. only executes if descore is true
    else if(routineIndex == 9) {
      if(DistancePos2.objectDistance(mm) >= distancePos2Unoccupied) {
        sorterLock(); // bring red ball up into top two rollers
        conveyorStop();
        wait(250, msec);
        conveyorSpin(false, 600); // outtake blue balls
        wait(850, msec);
        sorterSpin(false, 600);
        routineIndex = 10;
        startTime = Brain.Timer.value();
      }
    }
    else if(routineIndex == 10) {
      if((DistancePos1.objectDistance(mm) <= distancePos1Occupied 
      && (DistanceScoring.isObjectDetected() == false || DistanceScoring.objectDistance(mm) > distanceScoringBallSeen)) 
      || Brain.Timer.value() >= startTime + 1) {
        conveyorLock(); // stop once ball two is in position one
        sorterStop();
        intakesStop();
        routineIndex = 11;
      }
    }

    // strafe toward center goal on home row
    else if(routineIndex == 11) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = degreesToStrafe;
      driveArray[4] /* percentToAccel */ = 15;
      driveArray[5] /* percentToDecel */ = 15;
      strafeRight(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 12;
      }
    }

    // drive into center goal on home row
    else if(routineIndex == 12) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = 225;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 1;
      driveForward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          routineIndex = 14; // if there isn't a ball in the robot, skip scoring
        }
        else {
          routineIndex = 13;
        }
        alignWithGoal(100);
        intake();
        wait(300, msec);
        intakesStop();
        wait(200, msec);
      }
    }

    // score red ball in center goal on home row
    else if(routineIndex == 13) {
      scoreBallArray[0] /* stepsCompleted */ = false;
      scoreBallAutonNoColorID(scoreBallArray);

      if(scoreBallArray[0] /* stepsCompleted */ == true) {
        if(descore == true) { // based on the brain screen autonomous selection
          routineIndex = 14;
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          routineIndex = 16;
          drivetrainStop();
          trigCalcCenterToBall(39, 23);
          wait(100, msec);
          outtake(200);
        }
      }
    }

    // descore blue ball from center goal on home row. only executes if descore is true, or if it missed the red ball
    else if(routineIndex == 14) {
      collectBallArray[0] /* stepsCompleted */ = false;
      collectBallAutonNoColorID(collectBallArray);

      if(collectBallArray[0] /* stepsCompleted */ == true) {
        routineIndex = 15;
        alignWithGoal(150);
        outtake(200);
        startTime = Brain.Timer.value();
      }
    }
    else if(routineIndex == 15) {
      if(Brain.Timer.value() >= startTime + 0.2) {
        routineIndex = 16;
        intakesStop();
        drivetrainStop();
        trigCalcCenterToBall(39, 23);
        wait(100, msec);
        outtake(150);
        sorterSpin(false, 250); // outtake blue ball
        conveyorSpin(false, 250);
      }
    }

    // back up from center goal on home row
    else if(routineIndex == 16) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = degreesToReverse;
      driveArray[4] /* percentToAccel */ = 15;
      driveArray[5] /* percentToDecel */ = 15;
      driveBackward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 17;
        intakesStop();
        sorterStop();
        conveyorStop();
        wait(100, msec);
      }
    }

    // strafe to be aligned with red ball on home row line
    else if(routineIndex == 17) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 175;
      driveArray[3] /* degreesToTravel */ = degreesToStrafe;
      driveArray[4] /* percentToAccel */ = 25;
      driveArray[5] /* percentToDecel */ = 25;
      strafeRight(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 18;
        wait(100, msec);
      }
    }

    // drive forward and pick up red ball on home row line
    else if(routineIndex == 18) { // reset flags
      driveArray[0] /* stepsCompleted */ = false;
      collectBallArray[0] /* stepsCompleted */ = false;
      routineIndex = 19;
      startTime = Brain.Timer.value();
    }
    else if(routineIndex == 19) { // collecting and driving occur simultaneously
      if(collectBallArray[0] /* stepsCompleted */ == false) {
        collectBallAutonNoColorID(collectBallArray);
      }
      if(driveArray[0] /* stepsCompleted */ == false) {
        driveArray[0] /* stepsCompleted */ = false;
        driveArray[2] /* speed */ = 100;
        driveArray[3] /* degreesToTravel */ = 400;
        driveArray[4] /* percentToAccel */ = 30;
        driveArray[5] /* percentToDecel */ = 20;
        driveForward(driveArray);
      }
      if(driveArray[0] /* stepsCompleted */ == true && 
      (collectBallArray[0] /* stepsCompleted */ == true || Brain.Timer.value() >= startTime + 2.5)) {
        routineIndex = 20;
        intakesStop(); // only continues once both have completed, or if the waitTime failsafe for missing the ball is met
        count = 0;
        positionArray[3] /* evaluated */ = false;
      }
    }

    // back up a small amount
    else if(routineIndex == 20) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 250;
      driveArray[3] /* degreesToTravel */ = 290;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 20;
      driveBackward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 21;
        wait(5, msec);
      }
    }
    
    // turn to angle needed to align with far corner goal on home row
    else if(routineIndex == 21) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 100;
      turnArray[3] /* angle */ = -140;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 20;
      turnRightWithInertial(turnArray);

      if(turnArray[0] /* stepsCompleted */ == true) {
        routineIndex = 22;
      }
    }

    // drive into far corner goal on home row
    else if(routineIndex == 22) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = 650;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 1;
      driveForward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        alignWithGoal(100);
        intake();
        wait(300, msec);
        intakesStop();
        wait(200, msec);

        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          routineIndex = 24; // if it missed picking up the red ball, skip scoring
          descoreCorrection = true; // even if descore == false, descore to break a blue row if the red ball was missed
        }
        else {
          routineIndex = 23;
          descoreCorrection = false;
        } 
      }
    }
     
    // score one red ball in far corner goal on home row
    else if(routineIndex == 23) {
      scoreBallArray[0] /* stepsCompleted */ = false;
      scoreBallAutonNoColorID(scoreBallArray);

      if(scoreBallArray[0] /* stepsCompleted */ == true) {
        if(descore == true) { // based on the brain screen autonomous selection
          routineIndex = 24;
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          routineIndex = 26;
          drivetrainStop();
          trigCalcSecondCornerToBall(19, 56);
          outtake(200);
        }
      }
    }

    // descore first blue ball from far corner goal on home row. only executes if descore is true, or if it missed the red ball
    else if(routineIndex == 24) {
      collectBallArray[0] /* stepsCompleted */ = false;
      collectBallAutonNoColorID(collectBallArray);

      if(collectBallArray[0] /* stepsCompleted */ == true) {
        routineIndex = 25;
      }
    }

    // descore second blue ball from far corner goal on home row. only executes if descore is true, or if it missed the red ball
    else if(routineIndex == 25) {
      collectBallArray[0] /* stepsCompleted */ = false;
      collectBallAutonNoColorID(collectBallArray);

      if(collectBallArray[0] /* stepsCompleted */ == true) {
        routineIndex = 26;
        wait(200, msec);
        drivetrainStop();
        trigCalcSecondCornerToBall(19, 56);
        if(DistancePos0.objectDistance(mm) <= distancePos0Occupied && DistancePos1.objectDistance(mm) <= distancePos1Occupied) {
          outtake(200);
          wait(200, msec);
        }
      }
    }

    // back up from far corner goal on home row
    else if(routineIndex == 26) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = degreesToReverse;
      driveArray[4] /* percentToAccel */ = 15;
      driveArray[5] /* percentToDecel */ = 10;
      driveBackward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 27;
        if(descore == true || descoreCorrection == true) { // descore to break a blue row
          conveyorSpin(false, 600);
          sorterSpin(false, 600);
          outtake(200);
          wait(1000, msec);
          sorterStop();
          conveyorStop();
          intakesStop();
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          intakesStop();
        }
      }
    }

    // turn toward ball in front of fourth goal
    else if(routineIndex == 27) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 150;
      turnArray[3] /* angle */ = -1;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 25;
      turnRightWithInertial(turnArray);

      if(turnArray[0] /* stepsCompleted */ == true) {
        routineIndex = 28;
      }
    }

    // drive forward and pick up red ball in front of fourth goal
    else if(routineIndex == 28) { // reset flags
      driveArray[0] /* stepsCompleted */ = false;
      collectBallArray[0] /* stepsCompleted */ = false;
      routineIndex = 29;
      startTime = Brain.Timer.value();
    }
    else if(routineIndex == 29) { // collecting and driving occur simultaneously
      if(collectBallArray[0] /* stepsCompleted */ == false) {
        collectBallAutonNoColorID(collectBallArray);
      }
      if(driveArray[0] /* stepsCompleted */ == false) {
        driveArray[0] /* stepsCompleted */ = false;
        driveArray[2] /* speed */ = 125;
        driveArray[3] /* degreesToTravel */ = degreesToDrive;
        driveArray[4] /* percentToAccel */ = 20;
        driveArray[5] /* percentToDecel */ = 20;
        driveForward(driveArray);
      }
      if(driveArray[0] /* stepsCompleted */ == true && 
      (collectBallArray[0] /* stepsCompleted */ == true || Brain.Timer.value() >= startTime + 2)) {
        intakesStop(); // only continues once both have completed, or if the waitTime failsafe for missing the ball is met
        count = 0;
        positionArray[3] /* evaluated */ = false;
        routineIndex = 30;
        startingAngle = Inertial.yaw();
      }
    }

    // strafe a small amount away from the fourth goal
    else if(routineIndex == 30) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 100;
      driveArray[3] /* degreesToTravel */ = 55;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 20;
      strafeRight(driveArray);
      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 31;
      }
    }

    // turn into fourth goal
    else if(routineIndex == 31) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 150;
      turnArray[3] /* angle */ = -82;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 30;
      turnLeftWithInertial(turnArray);
      
      if(turnArray[0] /* stepsCompleted */ == true || startingAngle == -90) {
        routineIndex = 32;
      }
    }

    // drive into fourth goal
    else if(routineIndex == 32) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = 50;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 1;
      driveForward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        alignWithGoal(150);
        intake();
        wait(200, msec);
        intakesStop();
        wait(400, msec);

        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          routineIndex = 34; // if there isn't a ball in the robot, skip scoring
          descoreCorrection = true; // even if descore == false, descore to break a blue row if the red ball was missed
        }
        else {
          routineIndex = 33;
          descoreCorrection = false;
        }
      }
    }

    // score red ball in fourth goal
    else if(routineIndex == 33) {
      scoreBallArray[0] /* stepsCompleted */ = false;
      scoreBallAutonNoColorID(scoreBallArray);

      if(scoreBallArray[0] /* stepsCompleted */ == true) {
        if(descore == true) { // based on the brain screen autonomous selection
          routineIndex = 34;
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          routineIndex = 35;
          drivetrainStop();
        
          if(Inertial.yaw() < -92) { // correct angle
            DriveMotorLeftBack.spin(directionType::fwd,50,velocityUnits::rpm);
            DriveMotorRightBack.spin(directionType::rev,50,velocityUnits::rpm);
            waitUntil(Inertial.yaw() >= -92);
          }
          else if(Inertial.yaw() > -88) {
            DriveMotorLeftBack.spin(directionType::rev,50,velocityUnits::rpm);
            DriveMotorRightBack.spin(directionType::fwd,50,velocityUnits::rpm);
            waitUntil(Inertial.yaw() <= -88);
          }
          drivetrainStop();
          outtake(200);
        }
      }
    }

    // descore blue ball from fourth goal. only executes if descore is true, or if it missed the red ball
    else if(routineIndex == 34) {
      collectBallArray[0] /* stepsCompleted */ = false;
      collectBallAutonNoColorID(collectBallArray);

      if(collectBallArray[0] /* stepsCompleted */ == true) {
        routineIndex = 35;
        outtake(200);
        wait(200, msec);
        intakesStop();
        drivetrainStop();
        
        if(Inertial.yaw() < -92) { // correct angle
          DriveMotorLeftBack.spin(directionType::fwd,50,velocityUnits::rpm);
          DriveMotorRightBack.spin(directionType::rev,50,velocityUnits::rpm);
          waitUntil(Inertial.yaw() >= -92);
        }
        else if(Inertial.yaw() > -88) {
          DriveMotorLeftBack.spin(directionType::rev,50,velocityUnits::rpm);
          DriveMotorRightBack.spin(directionType::fwd,50,velocityUnits::rpm);
          waitUntil(Inertial.yaw() <= -88);
        }
        drivetrainStop();
        outtake(200);
      }
    }

    // back up from fourth goal
    else if(routineIndex == 35) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 150;
      driveArray[3] /* degreesToTravel */ = 450;
      driveArray[4] /* percentToAccel */ = 35;
      driveArray[5] /* percentToDecel */ = 20;
      driveBackward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 36;
        intakesStop();
      }
    }

    // turn toward ball on the far side's home row line
    else if(routineIndex == 36) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 150;
      turnArray[3] /* angle */ = -4;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 30;
      turnRightWithInertial(turnArray);

      if(turnArray[0] /* stepsCompleted */ == true) {
        routineIndex = 37;
        if(descore == true || descoreCorrection == true) { // descore to break a blue row
          sorterSpin(false, 600);
          conveyorSpin(true, 600);
          wait(500, msec);
          sorterStop();
          conveyorStop();
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          wait(100, msec);
        }
      }
    }

    // drive forward and pick up red ball on the far side's home row line
    else if(routineIndex == 37) { // reset flags
      driveArray[0] /* stepsCompleted */ = false;
      collectBallArray[0] /* stepsCompleted */ = false;
      routineIndex = 38;
      startTime = Brain.Timer.value();
    }
    else if(routineIndex == 38) { // collecting and driving occur simultaneously
      if(collectBallArray[0] /* stepsCompleted */ == false) {
        collectBallAutonNoColorID(collectBallArray);
      }
      if(driveArray[0] /* stepsCompleted */ == false) {
        driveArray[0] /* stepsCompleted */ = false;
        driveArray[2] /* speed */ = 150;
        driveArray[3] /* degreesToTravel */ = 1150;
        driveArray[4] /* percentToAccel */ = 20;
        driveArray[5] /* percentToDecel */ = 30;
        driveForward(driveArray);
      }
      if(driveArray[0] /* stepsCompleted */ == true && 
      (collectBallArray[0] /* stepsCompleted */ == true || Brain.Timer.value() >= startTime + 2.5)) {
        intakesStop();
        count = 0;
        positionArray[3] /* evaluated */ = false;
        routineIndex = 39;
      }
    }

    // back up a small amount
    else if(routineIndex == 39) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 100;
      driveArray[3] /* degreesToTravel */ = 100;
      driveArray[4] /* percentToAccel */ = 30;
      driveArray[5] /* percentToDecel */ = 20;
      driveBackward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 40;
        wait(100, msec);
      }
    }
    
    // turn to angle needed to align with fifth goal
    else if(routineIndex == 40) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 150;
      turnArray[3] /* angle */ = -36;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 30;
      turnLeftWithInertial(turnArray);

      if(turnArray[0] /* stepsCompleted */ == true) {
        routineIndex = 41;
      }
    }

    // drive into fifth goal
    else if(routineIndex == 41) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = 650;
      driveArray[4] /* percentToAccel */ = 40;
      driveArray[5] /* percentToDecel */ = 1;
      driveForward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          routineIndex = 43; // if it missed picking up the red ball, skip scoring
          descoreCorrection = true; // even if descore == false, descore to break a blue row if the red ball was missed
        }
        else {
          routineIndex = 42;
          descoreCorrection = false;
        }
        alignWithGoal(100);
        intake();
        wait(200, msec);
        intakesStop();
        wait(300, msec);
      }
    }
     
    // score one red ball in fifth goal
    else if(routineIndex == 42) {
      scoreBallArray[0] /* stepsCompleted */ = false;
      scoreBallAutonNoColorID(scoreBallArray);

      if(scoreBallArray[0] /* stepsCompleted */ == true) {
        if(descore == true) { // based on the brain screen autonomous selection
          routineIndex = 43;
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          routineIndex = 45;
          drivetrainStop();
          trigCalcThirdCornerToBall(62, 36);
          outtake(200);
        }
      }
    }

    // descore first blue ball from fifth goal. only executes if descore is true, or if it missed the red ball
    else if(routineIndex == 43) {
      collectBallArray[0] /* stepsCompleted */ = false;
      collectBallAutonNoColorID(collectBallArray);

      if(collectBallArray[0] /* stepsCompleted */ == true) {
        routineIndex = 44;
      }
    }

    // descore second blue ball from fifth goal. only executes if descore is true, or if it missed the red ball
    else if(routineIndex == 44) {
      collectBallArray[0] /* stepsCompleted */ = false;
      collectBallAutonNoColorID(collectBallArray);

      if(collectBallArray[0] /* stepsCompleted */ == true) {
        routineIndex = 45;
        drivetrainStop();
        trigCalcThirdCornerToBall(62, 36);
        if(DistancePos0.objectDistance(mm) <= distancePos0Occupied 
        && DistancePos1.objectDistance(mm) <= distancePos1Occupied) {
          outtake(200);
          wait(200, msec);
        }
      }
    }

    // back up from fifth goal (third corner)
    else if(routineIndex == 45) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = degreesToReverse;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 20;
      driveBackward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        if(descore == true || descoreCorrection == true) { // descore to break a blue row
          routineIndex = 46;
          conveyorSpin(false, 600);
          sorterSpin(false, 600);
          outtake(200);
          wait(1200, msec);
          sorterStop();
          conveyorStop();
          intakesStop();
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          routineIndex = 46;
          intakesStop();
        }
      }
    }

    // turn to angle needed to align with ball in front of sixth goal (center on far home row)
    else if(routineIndex == 46) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 150;
      turnArray[3] /* angle */ = 90;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 30;
      turnRightWithInertial(turnArray);

      if(turnArray[0] /* stepsCompleted */ == true) {
        routineIndex = 47;
      }
    }

    // drive forward and pick up red ball in front of sixth goal
    else if(routineIndex == 47) { // reset flags
      driveArray[0] /* stepsCompleted */ = false;
      collectBallArray[0] /* stepsCompleted */ = false;
      routineIndex = 48;
      startTime = Brain.Timer.value();
    }
    else if(routineIndex == 48) { // collecting and driving occur simultaneously
      if(collectBallArray[0] /* stepsCompleted */ == false) {
        collectBallAutonNoColorID(collectBallArray);
      }
      if(driveArray[0] /* stepsCompleted */ == false) {
        driveArray[0] /* stepsCompleted */ = false;
        driveArray[2] /* speed */ = 125;
        driveArray[3] /* degreesToTravel */ = degreesToDrive;
        driveArray[4] /* percentToAccel */ = 20;
        driveArray[5] /* percentToDecel */ = 20;
        driveForward(driveArray);
      }
      if(driveArray[0] /* stepsCompleted */ == true && 
      (collectBallArray[0] /* stepsCompleted */ == true || Brain.Timer.value() >= startTime + 2.4)) {
        intakesStop(); // only continues once both have completed, or if the waitTime failsafe for missing a ball is met
        count = 0;
        positionArray[3] /* evaluated */ = false;
        routineIndex = 49;
      }
    }

    // turn to sixth goal
    else if(routineIndex == 49) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 150;
      turnArray[3] /* angle */ = 9;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 30;
      turnLeftWithInertial(turnArray);

      if(turnArray[0] /* stepsCompleted */ == true) {
        routineIndex = 50;
      }
    }

    // drive into sixth goal
    else if(routineIndex == 50) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = 675;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 1;
      driveForward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          routineIndex = 52; // if it missed picking up the red ball, skip scoring
        }
        else {
          routineIndex = 51;
        }
        alignWithGoal(100);
        intake();
        wait(200, msec);
        intakesStop();
        wait(200, msec);
      }
    }

    // score red ball in sixth goal
    else if(routineIndex == 51) {
      scoreBallArray[0] /* stepsCompleted */ = false;
      scoreBallAutonNoColorID(scoreBallArray);

      if(scoreBallArray[0] /* stepsCompleted */ == true) {
        if(descore == true) { // based on the brain screen autonomous selection
          routineIndex = 52;
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          routineIndex = 54;
          drivetrainStop();
          trigCalcSixthGoalToBall(42, 23);
          wait(100, msec);
          outtake(200);
        }
      }
    }

    // descore blue ball from sixth goal. only executes if descore is true, or if it missed the red ball
    else if(routineIndex == 52) {
      collectBallArray[0] /* stepsCompleted */ = false;
      collectBallAutonNoColorID(collectBallArray);

      if(collectBallArray[0] /* stepsCompleted */ == true) {
        routineIndex = 53;
        alignWithGoal(150);
        outtake(200);
        startTime = Brain.Timer.value();
      }
    }
    else if(routineIndex == 53) {
      if(Brain.Timer.value() >= startTime + 0.2) {
        routineIndex = 54;
        intakesStop();
        drivetrainStop();
        trigCalcSixthGoalToBall(42, 23);
        wait(100, msec);
        outtake(150);
        sorterSpin(false, 250); // outtake blue ball
        conveyorSpin(false, 250);
      }
    }

    // back up from sixth goal
    else if(routineIndex == 54) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = degreesToReverse;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 20;
      driveBackward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 55;
        intakesStop();
        sorterStop();
        conveyorStop();
        wait(100, msec);
      }
    }

    // strafe to be aligned with red ball on far home row line
    else if(routineIndex == 55) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 165;
      driveArray[3] /* degreesToTravel */ = degreesToStrafe;
      driveArray[4] /* percentToAccel */ = 25;
      driveArray[5] /* percentToDecel */ = 20;
      strafeRight(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 56;
        wait(100, msec);
      }
    }

    // drive forward and pick up red ball on far home row line
    else if(routineIndex == 56) { // reset flags
      driveArray[0] /* stepsCompleted */ = false;
      collectBallArray[0] /* stepsCompleted */ = false;
      routineIndex = 57;
      startTime = Brain.Timer.value();
    }
    else if(routineIndex == 57) { // collecting and driving occur simultaneously
      if(collectBallArray[0] /* stepsCompleted */ == false) {
        collectBallAutonNoColorID(collectBallArray);
      }
      if(driveArray[0] /* stepsCompleted */ == false) {
        driveArray[0] /* stepsCompleted */ = false;
        driveArray[2] /* speed */ = 100;
        driveArray[3] /* degreesToTravel */ = 450;
        driveArray[4] /* percentToAccel */ = 30;
        driveArray[5] /* percentToDecel */ = 20;
        driveForward(driveArray);
      }
      if(driveArray[0] /* stepsCompleted */ == true && 
      (collectBallArray[0] /* stepsCompleted */ == true || Brain.Timer.value() >= startTime + 2.5)) {
        routineIndex = 58; // only continues if both are completed, or if the waitTime failsafe for missing a ball is met
        intakesStop();
        count = 0;
        positionArray[3] /* evaluated */ = false;
      }
    }

    // back up a small amount
    else if(routineIndex == 58) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = 260;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 20;
      driveBackward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 59;
        wait(5, msec);
      }
    }
    
    // turn to angle needed to align with seventh goal (second corner on far home row)
    else if(routineIndex == 59) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 145;
      turnArray[3] /* angle */ = 40;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 30;
      turnRightWithInertial(turnArray);

      if(turnArray[0] /* stepsCompleted */ == true) {
        routineIndex = 60;
      }
    }

    // drive into seventh goal
    else if(routineIndex == 60) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = 675;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 1;
      driveForward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        alignWithGoal(100);
        intake();
        wait(300, msec);
        intakesStop();
        wait(400, msec);

        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          routineIndex = 62; // if it missed picking up the red ball, skip scoring
          descoreCorrection = true; // even if descore == false, descore to break a blue row if the red ball was missed
        }
        else {
          routineIndex = 61;
          descoreCorrection = false;
        } 
      }
    }
     
    // score one red ball in seventh goal
    else if(routineIndex == 61) {
      scoreBallArray[0] /* stepsCompleted */ = false;
      scoreBallAutonNoColorID(scoreBallArray);

      if(scoreBallArray[0] /* stepsCompleted */ == true) {
        if(descore == true) { // based on the brain screen autonomous selection
          routineIndex = 62;
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          routineIndex = 64;
          drivetrainStop();
          trigCalcFourthCornerToBall(16, 56);
          outtake(200);
        }
      }
    }

    // descore first blue ball from seventh goal. only executes if descore is true, or if it missed the red ball
    else if(routineIndex == 62) {
      collectBallArray[0] /* stepsCompleted */ = false;
      collectBallAutonNoColorID(collectBallArray);

      if(collectBallArray[0] /* stepsCompleted */ == true) {
        routineIndex = 63;
      }
    }

    // descore second blue ball from seventh goal. only executes if descore is true, or if it missed the red ball
    else if(routineIndex == 63) {
      collectBallArray[0] /* stepsCompleted */ = false;
      collectBallAutonNoColorID(collectBallArray);

      if(collectBallArray[0] /* stepsCompleted */ == true) {
        routineIndex = 64;
        wait(200, msec);
        drivetrainStop();
        trigCalcFourthCornerToBall(16, 56);
        if(DistancePos0.objectDistance(mm) <= distancePos0Occupied && DistancePos1.objectDistance(mm) <= distancePos1Occupied) {
          outtake(200);
          wait(200, msec);
        }
      }
    }

    // back up from seventh goal
    else if(routineIndex == 64) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = degreesToReverse;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 10;
      driveBackward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 65;
        if(descore == true || descoreCorrection == true) { // descore to break a blue row
          conveyorSpin(false, 600);
          sorterSpin(false, 600);
          outtake(200);
          wait(1000, msec);
          sorterStop();
          conveyorStop();
          intakesStop();
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          intakesStop();
        }
      }
    }

    // turn toward ball in front of eighth goal (last unscored goal other than the middle one)
    else if(routineIndex == 65) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 150;
      turnArray[3] /* angle */ = 175;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 20;
      turnRightWithInertial(turnArray);

      if(turnArray[0] /* stepsCompleted */ == true) {
        routineIndex = 66;
      }
    }

    // drive forward and pick up red ball in front of eighth goal
    else if(routineIndex == 66) { // reset flags
      driveArray[0] /* stepsCompleted */ = false;
      collectBallArray[0] /* stepsCompleted */ = false;
      routineIndex = 67;
      startTime = Brain.Timer.value();
    }
    else if(routineIndex == 67) { // collecting and driving occur simultaneously
      if(collectBallArray[0] /* stepsCompleted */ == false) {
        collectBallAutonNoColorID(collectBallArray);
      }
      if(driveArray[0] /* stepsCompleted */ == false) {
        driveArray[0] /* stepsCompleted */ = false;
        driveArray[2] /* speed */ = 115;
        driveArray[3] /* degreesToTravel */ = degreesToDrive;
        driveArray[4] /* percentToAccel */ = 20;
        driveArray[5] /* percentToDecel */ = 20;
        driveForward(driveArray);
      }
      if(driveArray[0] /* stepsCompleted */ == true && 
      (collectBallArray[0] /* stepsCompleted */ == true || Brain.Timer.value() >= startTime + 2)) {
        intakesStop(); // only continues if both are completed, or if the waitTime failsafe is met
        count = 0;
        positionArray[3] /* evaluated */ = false;
        routineIndex = 68;
        startingAngle = Inertial.yaw();
      }
    }

    // strafe a small amount away from the eighth goal
    else if(routineIndex == 68) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 100;
      driveArray[3] /* degreesToTravel */ = 55;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 20;
      strafeRight(driveArray);
      
      if(driveArray[0] /* stepsCompleted */ == true) {
        routineIndex = 69;
        wait(100, msec);
      }
    }

    // turn into eighth goal
    else if(routineIndex == 69) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 150;
      turnArray[3] /* angle */ = 98;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 30;
      turnLeftWithInertial(turnArray);
      
      if(turnArray[0] /* stepsCompleted */ == true) {
        routineIndex = 70;
      }
    }

    // drive into eighth goal
    else if(routineIndex == 70) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = 75;
      driveArray[4] /* percentToAccel */ = 20;
      driveArray[5] /* percentToDecel */ = 1;
      driveForward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        alignWithGoal(150);
        intake();
        wait(200, msec);
        intakesStop();
        wait(200, msec);

        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          routineIndex = 72; // if there isn't a ball in the robot, skip scoring
        }
        else {
          routineIndex = 71;
        }
      }
    }

    // score red ball in eighth goal
    else if(routineIndex == 71) {
      scoreBallArray[0] /* stepsCompleted */ = false;
      scoreBallAutonNoColorID(scoreBallArray);

      if(scoreBallArray[0] /* stepsCompleted */ == true) {
        if(descore == true) { // based on the brain screen autonomous selection
          routineIndex = 72;
        }
        else if(descore == false) { // based on the brain screen autonomous selection
          routineIndex = 73;
          if(Inertial.yaw() < 90) {
            DriveMotorLeftBack.spin(directionType::fwd,50,velocityUnits::rpm);
            DriveMotorRightBack.spin(directionType::rev,50,velocityUnits::rpm);
            waitUntil(Inertial.yaw() >= 90);
          }
          else if(Inertial.yaw() > 91) {
            DriveMotorLeftBack.spin(directionType::rev,50,velocityUnits::rpm);
            DriveMotorRightBack.spin(directionType::fwd,50,velocityUnits::rpm);
            waitUntil(Inertial.yaw() <= 91);
          }
          drivetrainStop();
          outtake(200);
        }
      }
    }

    // descore blue ball from eighth goal. only executes if descore is true
    else if(routineIndex == 72) {
      if(descore == true) { // based on the brain screen autonomous selection
        collectBallArray[0] /* stepsCompleted */ = false;
        collectBallAutonNoColorID(collectBallArray);
      }
      else { // don't descore the last goal unless descore == true, even if it misses the red ball
        collectBallArray[0] /* stepsCompleted */ = true;
      }

      if(collectBallArray[0] /* stepsCompleted */ == true) {
        routineIndex = 73;
        if(Inertial.yaw() < 87) {
          DriveMotorLeftBack.spin(directionType::fwd,50,velocityUnits::rpm);
          DriveMotorRightBack.spin(directionType::rev,50,velocityUnits::rpm);
          waitUntil(Inertial.yaw() >= 87);
        }
        else if(Inertial.yaw() > 88) {
          DriveMotorLeftBack.spin(directionType::rev,50,velocityUnits::rpm);
          DriveMotorRightBack.spin(directionType::fwd,50,velocityUnits::rpm);
          waitUntil(Inertial.yaw() <= 88);
        }
        drivetrainStop();
        outtake(200);
      }
    }

    // back up from eighth goal
    else if(routineIndex == 73) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = 200;
      driveArray[4] /* percentToAccel */ = 10;
      driveArray[5] /* percentToDecel */ = 10;
      driveBackward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        intakesStop();
        routineIndex = 74;
      }
    }

    // turn around to face red ball in front of center goal
    else if(routineIndex == 74) {
      turnArray[0] /* stepsComplete */ = false;
      turnArray[2] /* speed */ = 175;
      turnArray[3] /* angle */ = -91;
      turnArray[4] /* percentToAccel */ = 10;
      turnArray[5] /* percentToDecel */ = 25;
      turnLeftWithInertial(turnArray);
      
      if(turnArray[0] /* stepsCompleted */ == true) {
        routineIndex = 75;
      }
    }

    // drive forward and pick up red ball in front of center goal
    else if(routineIndex == 75) { // reset flags
      collectBallArray[0] /* stepsCompleted */ = false;
      driveArray[0] /* stepsCompleted */ = false;
      routineIndex = 76;
      startTime = Brain.Timer.value();
    }
    else if(routineIndex == 76) { // collecting and driving occur simultaneously
      if(collectBallArray[0] /* stepsCompleted */ == false) {
        collectBallAutonNoColorID(collectBallArray);
      }
      if(driveArray[0] /* stepsCompleted */ == false) {
        driveArray[2] /* speed */ = 110;
        driveArray[3] /* degreesToTravel */ = 400;
        driveArray[4] /* percentToAccel */ = 15;
        driveArray[5] /* percentToDecel */ = 15;
        driveForward(driveArray);
      }

      if(driveArray[0] /* stepsCompleted */ == true && 
      (collectBallArray[0] /* stepsCompleted */ == true || Brain.Timer.value() >= startTime + 2)) {
        routineIndex = 77; // only continues once both are complete, or the waitTime failsafe is met
        intakesStop();
        count = 0;
        positionArray[3] /* evaluated */ = false;
      }
    }

    // angle slightly for an optimal position for descoring the center goal
    else if(routineIndex == 77) {
      // turnArray[0] /* stepsComplete */ = false;
      // turnArray[2] /* speed */ = 80;
      // turnArray[3] /* angle */ = -90;
      // turnArray[4] /* percentToAccel */ = 10;
      // turnArray[5] /* percentToDecel */ = 20;
      // turnLeftWithInertial(turnArray);
      
      // if(turnArray[0] /* stepsCompleted */ == true) {
        routineIndex = 78;
      // }
    }

    // drive into center goal and push a blue ball out
    else if(routineIndex == 78) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = 650;
      driveArray[4] /* percentToAccel */ = 15;
      driveArray[5] /* percentToDecel */ = 1;
      driveForward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        if(DistancePos1.objectDistance(mm) > distancePos1Unoccupied 
        && DistancePos2.objectDistance(mm) > distancePos2Unoccupied) {
          routineIndex = 80; // if there isn't a ball in the robot, skip scoring
        }
        else {
          routineIndex = 79;
          alignWithGoal(100);
        }
      }
    }

    // score red ball in center goal
    else if(routineIndex == 79) {
      scoreBallArray[0] /* stepsCompleted */ = false;
      scoreBallAutonNoColorID(scoreBallArray);

      if(scoreBallArray[0] /* stepsCompleted */ == true) {
        routineIndex = 80;
        drivetrainStop();
      }
    }

    // back up from center goal
    else if(routineIndex == 80) {
      driveArray[0] /* stepsCompleted */ = false;
      driveArray[2] /* speed */ = 200;
      driveArray[3] /* degreesToTravel */ = 400;
      driveArray[4] /* percentToAccel */ = 5;
      driveArray[5] /* percentToDecel */ = 10;
      driveBackward(driveArray);

      if(driveArray[0] /* stepsCompleted */ == true) {
        break; // end of the autonomous routine, break out of the while loop
      }
    }    
  }
  drivetrainStop();
  intakesStop();
  conveyorStop();
  sorterStop();
}

void usercontrol( void ) {
  // used for controller vibrations in driver skills to signal that time is almost out
  Brain.Timer.reset();
  int rumbleCount = 0;

  /* These are the variables and arrays passed into the parameters of my macros and functions */
  bool driveActive = false;
  bool turnActive = false;
  bool strafeActive = false;
  int drivetrainArray[3] = {driveActive, turnActive, strafeActive};

  bool collectBallActive = false;
  int collectBallArray[1] = {collectBallActive};

  bool scoreBallActive = false;
  int scoreBallArray[1] = {scoreBallActive};
  allowManualIntakingWhileScoring = false;
  
  bool scoreAndDescoreActive = false;
  int scoreAndDescoreArray[1] = {scoreAndDescoreActive};

  bool disposeBallActive = false;
  int disposeBallArray[1] = {disposeBallActive};

  bool intakesManualActive = false;
  int intakesManualArray[1] = {intakesManualActive};

  bool conveyorAndSorterManualActive = false;
  int conveyorAndSorterManualArray[1] = {conveyorAndSorterManualActive};

  /* This while loop only runs once at the beginning of the match */
  bool unstowed = false;
  while(unstowed == false) { // the intakes must be unstowed before any macros or manual controls can be used
    drivetrainMotion(drivetrainArray); // the wheels may be controlled when stowed
    if(Controller.ButtonLeft.pressing() == true) {
      drivetrainStop();
      unstow();
      unstowed = true;
    }
  }

  while(true) {
    // warning that time is almost out
    if(Brain.Timer.value() >= 58 && Brain.Timer.value() < 58.5 && rumbleCount == 0) {
      Controller.rumble(".");
      rumbleCount = 1; // vibrate once when there are two seconds left
    }
    else if(Brain.Timer.value() >= 59 && Brain.Timer.value() < 59.5 && rumbleCount == 1) {
      Controller.rumble(".");
      rumbleCount = 2; // vibrate once when there is one second left
    }
    else if(Brain.Timer.value() >= 59.5 && Brain.Timer.value() < 59.8 && rumbleCount == 2) {
      Controller.rumble(".");
      rumbleCount = 3; // vibrate once when there is half a second left
    }

    /* AUTO-SORTING TOGGLE: red, blue, off */
    if(Controller.ButtonY.pressing() == true) {
      autoSortToggle();
    }

    /* DRIVETRAIN MOVEMENT: driving, turning, strafing */
    drivetrainMotion(drivetrainArray);

    /* AUTO-SORTING AND MACRO FAILSAFE: stop and reset everything */
    if(Controller.ButtonA.pressing() == true) { 
      intakesStop();
      conveyorStop();
      sorterStop();
      count = 0;
      collectBallArray[0] = false;
      scoreBallArray[0] = false;
      scoreTwo = false;
      conditionOne = false;
      conditionTwo = false;
      allowManualIntakingWhileScoring = false;
      scoreAndDescoreArray[0] = false;
      disposeBallArray[0] = false;
      intakesManualArray[0] = false;
      conveyorAndSorterManualArray[0] = false;
      positionArray[3] /* evaluated */ = false;
      opticalLightOff();
      if(correctPositionOneHalf == true) {
        correctPositionOneHalf = false;
        positionOneHalfOccupied = false;
      }
    }

    /* MACROS: special conditions for more versatile scoring */

    if(allowManualIntakingWhileScoring == true) {
      scoreBall(scoreBallArray); // allows manual intaking / outtaking when scoring as long as the 
      intakesManual(intakesManualArray); // intakes are not in use in the scoring macro
    }

    if(scoreTwo == true && scoreBallArray[0] == false) {
      scoreBallArray[0] = true; // in case L1 and L2 aren't pressed exactly at the same time
    }

    /* MACROS: collecting, scoring, descoring, disposing */

    // collectBall can only be called when no other macros or manual controls are in use
    if(scoreBallArray[0] == false && scoreAndDescoreArray[0] == false
    && intakesManualArray[0] == false && conveyorAndSorterManualArray[0] == false
    && disposeBallArray[0] == false) {
      collectBall(collectBallArray); 
    }

    // scoreBall can only be called when no other macros or manual controls are in use
    // the intakes may be used at the same time with the special condition above
    if(collectBallArray[0] == false && scoreAndDescoreArray[0] == false 
    && intakesManualArray[0] == false && conveyorAndSorterManualArray[0] == false 
    && disposeBallArray[0] == false) {
      scoreBall(scoreBallArray);
    }

    // scoreAndDescore can only be called when no other macros or manual controls are in use
    if(collectBallArray[0] == false && scoreBallArray[0] == false 
    && intakesManualArray[0] == false && conveyorAndSorterManualArray[0] == false
    && disposeBallArray[0] == false) {
      scoreAndDescore(scoreAndDescoreArray);
    }

    // disposeBall can only be called when no other macros or manual controls are in use
    if(collectBallArray[0] == false && scoreBallArray[0] == false 
    && scoreAndDescoreArray[0] == false && intakesManualArray[0] == false 
    && conveyorAndSorterManualArray[0] == false) {
      disposeBall(disposeBallArray);
    }

    /* MANUAL CONTROLS: intaking, outtaking, scoring, disposing, reversing */

    // the manual intake controls can only be called when no macros are in use
    // the manual controls for the conveyor and sorter may be used at the same time as the manual intake controls
    if(collectBallArray[0] == false && scoreBallArray[0] == false
    && scoreAndDescoreArray[0] == false && disposeBallArray[0] == false) {
      intakesManual(intakesManualArray);
    }

    // the manual conveyor and sorter controls can only be called when no macros are in use
    // the manual controls for the intakes may be used at the same time as the manual conveyor and sorter controls
    if(collectBallArray[0] == false && scoreBallArray[0] == false 
    && scoreAndDescoreArray[0] == false && disposeBallArray[0] == false) {
      conveyorAndSorterManual(conveyorAndSorterManualArray);
    }
  }
}

int main() {
  pre_auton();

  Competiton.autonomous( autonomous );
  Competiton.drivercontrol( usercontrol );
                      
  while(true) {
    vex::task::sleep(100);
  }         
}