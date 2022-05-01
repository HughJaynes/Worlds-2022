#include "main.h"

bool tStart = true;
bool tPos = false;
double lTarget = 0;
bool baseMove = true;


double baseCutoff = 0;
double targetLeft = 0, targetRight = 0;
double targBearing = 0;
double errorBearing = 0, errorLeft = 0, errorRight = 0;
double powerLeft = 0, powerRight = 0;
double moveKp = DEFAULTMOVEKP;
double moveKd = DEFAULTMOVEKD;


void liftControl (void* lPosPointer) {

	Motor LI (LIPORT);

		LI.tare_position();

    while (true) {
        LI.move(lTarget - LI.get_position() * LIFTKP);
        delay(5);
    }

}


void moveLift(double lPosTarg) {
	lTarget = lPosTarg;
}

void toggleSwitch (){tPos = !tPos;}
void toggleTilter(void *ignore) {

	ADIDigitalOut T1 (T1PISTON);
	ADIDigitalOut T2 (T2PISTON);
	ADIDigitalOut TC (TCPISTON);
while(true){
	if(tPos){
		T1.set_value(HIGH);
		T2.set_value(HIGH);
		delay(1000);
		TC.set_value(HIGH);
	}else{
		TC.set_value(LOW);
		T1.set_value(LOW);
		T2.set_value(LOW);
	}
 }
}


void baseControl(void * ignore) {

  Motor FL (FLPORT);
  Motor FR (FRPORT);
  Motor ML (MLPORT);
  Motor MR (MRPORT);
  Motor BL (BLPORT);
  Motor BR (BRPORT);
	Imu IMU (IMUPORT);

	double previousLeft = 0, previousRight = 0;
	while (competition::is_autonomous()) {
	  if (!IMU.is_calibrating()) {
        if (baseMove) {
          errorLeft = targetLeft - FL.get_position();
          errorRight = targetRight - FR.get_position();

          double derivativeLeft = errorLeft - previousLeft;
          double derivativeRight = errorRight - previousRight;

          previousLeft = errorLeft;
          previousRight = errorRight;

          double powerLeft = (errorLeft * moveKp) + (derivativeLeft * moveKd);
          double powerRight = powerLeft;

				}else {
					double prevErrorEncdL = 0, prevErrorEncdR = 0, prevErrorBearing = 0;
					errorBearing = targBearing - bearing;
        	double deltaErrorBearing = errorBearing - prevErrorBearing;

					double powerLeft = (errorBearing * moveKp) + (deltaErrorBearing * moveKd);
					double powerRight = -powerLeft;

        	prevErrorBearing = errorBearing;
				}
				double deltapowerLeft = targetLeft - powerLeft;
				powerLeft += abscap(powerLeft, MAX_POW);
				double deltapowerRight = targetRight - powerRight;
				powerRight += abscap(powerRight, MAX_POW);

				FL.move(powerLeft);
				ML.move(powerLeft);
				BL.move(powerLeft);
				FR.move(powerRight);
				MR.move(powerRight);
				BR.move(powerRight);
				delay(5);
			}
	}
}

void moveBase(double targ, double cutoff, double baseKp, double baseKd) {
	baseCutoff = cutoff;
	targetLeft = targ/INPERDEG;
	targetRight = targ/INPERDEG;
	baseKp = moveKp;
	baseKd = moveKd;
	baseMove = true;
}

void moveBase(double targ, double cutoff){
	moveBase(targ, cutoff, DEFAULTMOVEKD, DEFAULTMOVEKP);
}

void turnBase(double turn_bearing, double cutoff, double baseKp, double baseKd){
	baseMove = false;
	targBearing = turn_bearing;
	baseKp = moveKp;
	baseKd = moveKd;
}

void turnBase(double bearing, double cutoff){
	turnBase(bearing, cutoff, DEFAULTMOVEKD, DEFAULTMOVEKP);
}
