#include "main.h"
#include "auton.hpp"


void autonCode() {
  Motor FL(FLPort);
  Motor BL(BLPort);
  Motor FR(FRPort);
  Motor BR(BRPort);
  Motor AL(ALPort);
  Motor AR(ARPort);
  Motor CL(CLPort);
  Motor LF(LFPort);

// Deposit Neutral 1
  LF.move_absolute(740,127);
  LF.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  delay(1000);
  baseMove(-630,-630,1300,0.2,0.2);
  delay(1300);
  LF.move_absolute(100,127);
  LF.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  delay(1000);
  AL.move_absolute(513,127);
  AR.move_absolute(513,127);
  AL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  AR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  baseMove(100,100,1000,DEFAULTDRIVEKP,DEFAULTDRIVEKD);
  delay(1000);
  AL.move_absolute(0,127);
  AR.move_absolute(0,127);
  AL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  AR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  delay(1000);
  baseMove(870,-60,750,DEFAULTDRIVEKP,DEFAULTDRIVEKD);
  delay(750);
  baseMove(2700,2700,4000,0.3,0.3);
  delay(4000);
  CL.move_absolute(CLAWDOWN,127);
  CL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  delay(750);
  AL.move_absolute(370,127);
  AR.move_absolute(370,127);
  AL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  AR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  baseMove(500,-500,1500,0.3,0.2);
  delay(1500);
  baseMove(300,300,1500,0.3,0.2);
  delay(1500);
  baseMove(-60,850,1000,0.3,0.3);
  delay(1000);
  AL.move_absolute(340,127);
  AR.move_absolute(340,127);
  AL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  AR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  delay(1000);
  CL.move_absolute(CLAWUP,127);
  delay(1000);
  baseMove(-200,-200,1000,0.3,0.2);
  delay(1000);
//   baseMove(700,700,750,0.3,0.4);
//   delay(750);
//   AL.move_absolute(380,127);
//   AR.move_absolute(380,127);
//   delay(150);
//   baseMove(780,-50,1000,0.2,0.3);
//   delay(1000);
//   baseMove(420,420,750,0.2,0.1);
//   delay(1000);
//   CL.move_absolute(CLAWUP,127);
//   delay(500);
//
// // Neutral 2
//   baseMove(-150,-150,500,DEFAULTDRIVEKP,DEFAULTDRIVEKD);
//   delay(500);
//   baseMove(-850,850,1000,0.15,0.3);
//   AL.move_absolute(ARMLOW,127);
//   AR.move_absolute(ARMLOW,127);
//   delay(1200);
//   baseMove(570,570,1000,0.2,0.4);
//   delay(1000);
//   CL.move_absolute(CLAWDOWN,127);
//   delay(300);
//   baseMove(-150,-150,500,DEFAULTDRIVEKP,DEFAULTDRIVEKD);
//   delay(500);
//   baseMove(-850,850,1000,0.15,0.3);
//   AL.move_absolute(380,127);
//   AR.move_absolute(380,127);
//   delay(1000);
//   baseMove(460,420,1750,0.2,0.25);
//   delay(1750);
//   baseMove(200,-200,500,0.2,0.3);
//   delay(750);
//   CL.move_absolute(CLAWUP,127);
}

void lrtAuton2() {
  Motor CL (CLPort);
  Motor LF (LFPort);
  Motor AL (ALPort);
  Motor AR (ARPort);

  LF.move_absolute(720,127);
  LF.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  delay(1000);
  baseMove(-650,-650,1500,0.2,0.9);
  delay(1400);
  LF.move_absolute(75,127);
  LF.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  delay(1000);
  baseMove(75,75,1000,0.3,0.3);
  delay(1000);
  baseMove(1150,50,1500,0.3,0.3);
  delay(1500);
  baseMove(2380,2380,2500,0.3,0.3);
  delay(2500);
}

void lrtAuton() {
  baseMove(2800,2800,2500,0.3,0.3);
  delay(2500);
  baseMove(0,-690,1000,0.3,0.3);
  delay(1000);
  baseMove(1060,1060,1500,0.3,0.3);
  delay(1500);
  baseMove(430,-430,1000,0.3,0.3);
  delay(1000);
  baseMove(1400,1400,2000,0.3,0.7);
  delay(400);
  delay(2000);
}

void lControl (int target, int timeout) {
	Motor LF (LFPort);
    LF.tare_position();
    int start = millis();
    while (millis() > timeout) {
        LF.move((target - LF.get_position()) * 0.5);
    }
}
