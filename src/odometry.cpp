#include "main.h"

// VARIABLES
double leftEncoderDiff = 0;
double rightEncoderDiff = 0;
double backEncoderDiff = 0;

double currentBearing = 0;
double globalY = 0;
double globalX = 0;

double targetRotate = 0;
double rotateKp = 0;
double rotateKd = 0;
double cutoffRotate = 0;
bool baseRotateState = false;

double targetXMove = 0;
double targetYMove = 0;
double moveKp = 0;
double moveKd = 0;
double cutoffMove = 0;
bool baseMoveState = false;

// ODOMETRY
void baseOdometry(void * ignore) {
  // Set variables
  double prevLeftEncoder = 0;
  double prevRightEncoder = 0;
  double prevBackEncoder = 0;

  while (true) {
    // Distance moved by encoders
    leftEncoderDiff = prevLeftEncoder;
    rightEncoderDiff = prevLeftEncoder;
    backEncoderDiff = prevLeftEncoder;
    double theta = (leftEncoderDiff - rightEncoderDiff)/(SL + SR);
    currentBearing += theta;

    // Update prev variables
    prevLeftEncoder = 0;
    prevRightEncoder = 0;
    prevBackEncoder = 0;

    // Calculate local offset
    double arcRadius = (rightEncoderDiff/theta) + SR;
    double localXChange = 2 * (sin(theta/2)) * ((backEncoderDiff/theta) + SS);
    double localYChange = 2 * (sin(theta/2)) * arcRadius;

    // Calculate global offset by avg rotation
    double avgRotation = currentBearing - (theta/2);
    double globalXChange = (cos(-avgRotation) * localXChange) - (sin(-avgRotation) * localYChange);
    double globalYChange = (sin(-avgRotation) * localXChange) + (cos(-avgRotation) * localYChange);
    globalX += globalXChange;
    globalY += globalYChange;

    delay(5);
  }
}


// ROTATE CODE
void baseControl(void * ignore) {
  Motor FL (FL);
  Motor FR (FR);
  Motor ML (ML);
  Motor MR (MR);
  Motor BL (BL);
  Motor BR (BR);

  // ROTATE CODE
  if (competition::is_autonomous()) {
    if (baseRotateState) {
      double targetRotate = targetRotate * (atan(1)*4/180);
      double prevErrorRotate = 0;
      double startTime = millis();
      // PD loop
      while (fabs(targetRotate - currentBearing) > ROTATEERRORMARGIN && millis() - startTime <= cutoffRotate) {

        // Calculate powers
        double errorRotate = targetRotate - currentBearing;
        double rotateProportional = errorRotate * ROTATEKP;
        double rotateDerivative = (errorRotate - prevErrorRotate) * ROTATEKD;
        double rotatePower = rotateProportional + rotateDerivative;
        double prevRotateError = errorRotate;

        if (rotatePower > ROTATEMAXPOWER) {
          rotatePower = ROTATEMAXPOWER;
        }
        if (rotatePower < ROTATEMAXPOWER * -1) {
          rotatePower = ROTATEMAXPOWER * -1;
        }

        // Move motors
        FL.move(rotatePower);
        ML.move(rotatePower);
        BL.move(rotatePower);
        FR.move(-rotatePower);
        MR.move(-rotatePower);
        BR.move(-rotatePower);

        delay(5);
      }
      baseRotateState = false;
    }

    else if (baseMoveState) {

      // Calculate powers
      double moveXError = targetXMove - globalX;
      double moveYError = targetYMove - globalY;
      double straightError = sqrt(pow(moveXError,2) + pow(moveXError,2));
      double startTime = millis();
      while (fabs(straightError) > MOVEERRORMARGIN && millis() - startTime <= cutoffMove) {
        double moveXError = targetXMove - globalX;
        double moveYError = targetYMove - globalY;
        double straightError = sqrt(pow(moveXError,2) + pow(moveXError,2));
        double moveKp


      delay(5);
      }
    }

    else {
      FL.move(0);
      ML.move(0);
      BL.move(0);
      FR.move(0);
      MR.move(0);
      BR.move(0);
    }
  }
}


void rotateBase(double rotateBaseDegrees, double rotateBaseCutoff, double rotateKpInput = ROTATEKP, double rotateKdInput = ROTATEKD) {
  targetRotate = rotateBaseDegrees * ((atan(1)*4)/180);
  cutoffRotate = rotateBaseCutoff;
  rotateKp = rotateKpInput;
  rotateKd = rotateKdInput;
  baseRotateState = true;
}

void pointBase(double pointX, double pointY, double pointCutoff, double pointKp = ROTATEKP, double pointKd = ROTATEKD) {
  double pointXError = pointX - globalX;
  double pointYError = pointY - globalY;
  double targetPoint = atan(pointXError/pointYError) * 180/atan(1)*4;
  rotateBase(targetPoint, pointCutoff, pointKp, pointKd);
}

void moveToBase(double moveTargetX, double moveTargetY, double moveToCutoff, double moveToKp = MOVEKP, double moveToKd = MOVEKD) {
}
