#include "main.h"
#include "mechLib.hpp"

double targetL = 0;
double targetR = 0;
double timeout = 0;
double driveKp = 0;
double driveKd = 0;
double errorL = 0, errorR = 0;
double derivativeL = 0, derivativeR = 0;
double powerL = 0, powerR = 0;
double previousL = 0, previousR = 0;
double height = 0;
double clawPosition = 0;



void armControl(double height, double armKp) {

  Motor AL(ALPort);
  Motor AR(ARPort);

  double errorAL = height - AL.get_position();
  double errorAR = height - AR.get_position();
  if (fabs(errorAL) > 5 && fabs(errorAR) > 5) {
    double powerAL = errorAL * armKp;
    double powerAR = errorAR * armKp;
    if (powerAL > 127) {
      powerAL = 127;
    }
    if (powerAL < -127) {
      powerAL = -127;
    }
    if (powerAR > 127) {
      powerAR = 127;
    }
    if (powerAR < -127) {
      powerAR = -127;
    }
    AL.move(powerAL);
    AR.move(powerAR);
  }
}

void clawControl(double clawPosition) {

    Motor CL(CLPort);

    double clawError = clawPosition - CL.get_position();

    if (fabs(clawError) > 10) {
      double powerClaw = clawError * CLAWKP;
      if (powerClaw > 127) {
        powerClaw = 127;
      }
      if (powerClaw < -127) {
        powerClaw = -127;
      }
      CL.move(powerClaw);
    }

}

void drivePD(void * ignore) {

  Motor FL(FLPort);
	Motor BL(BLPort);
	Motor FR(FRPort);
	Motor BR(BRPort);

  while(true){
      errorL = targetL - BL.get_position();
      errorR = targetR - BR.get_position();
      while (fabs(errorL) > 10 || fabs(errorR) > 10) {
        if (competition::is_autonomous()) {
          errorL = targetL - BL.get_position();
          errorR = targetR - BR.get_position();
          derivativeL = errorL - previousL;
          derivativeR = errorR - previousR;
          previousL = errorL;
          previousR = errorR;
          powerL = (errorL * driveKp) + (derivativeL * driveKd);
          powerR = (errorR * driveKp) + (derivativeR * driveKd);
          if (powerL > 127) {
            powerL = 127;
          }
          if (powerL < -127) {
            powerL = -127;
          }
          if (powerR > 127) {
            powerR = 127;
          }
          if (powerR < -127) {
            powerR = -127;
          }
          FL.move(powerL);
          BL.move(powerL);
          FR.move(powerR);
          BR.move(powerR);
          delay(5);
          timeout = timeout - 5;
        }
    }
  }
}

void baseMove(double targL, double targR, double cutoff, double kP = DEFAULTDRIVEKP, double kD = DEFAULTDRIVEKD) {
  Motor FL(FLPort);
  Motor BL(BLPort);
  Motor FR(FRPort);
  Motor BR(BRPort);

  FL.tare_position();
  BL.tare_position();
  FR.tare_position();
  BR.tare_position();

  targetL = targL;
  targetR = targR;
  driveKd = kD;
  driveKp = kP;
  timeout = cutoff;
}