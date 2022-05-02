#include "main.h"

void initialize() {

	Motor FL(FLPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor BL(BLPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor FR(FRPort, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	Motor BR(BRPort, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	Motor AL(ALPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor AR(ARPort, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	Motor CL(CLPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor LF(LFPort, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);
	Task drivePDTask(drivePD, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "My Task");

}

void disabled() {}


void competition_initialize() {}


void autonomous() {

	Motor FL(FLPort);
	Motor BL(BLPort);
	Motor FR(FRPort);
	Motor BR(BRPort);
	Motor AL(ALPort);
	Motor AR(ARPort);
	Motor CL(CLPort);
	Motor LF(LFPort);

	AL.tare_position();
	AR.tare_position();
	CL.tare_position();
	LF.tare_position();

	lrtAuton2();
}


void opcontrol() {
	double targetArm = 0;
	double targetClaw = 0;
	int armPos = 0;
	double changeArmKp = ARMKP;
	bool clawState = false;
	int liftDir = 0;

// Initializing
	Motor FL(FLPort);
	Motor BL(BLPort);
	Motor FR(FRPort);
	Motor BR(BRPort);
	Motor AL(ALPort);
	Motor AR(ARPort);
	Motor CL(CLPort);
	Motor LF(LFPort);

	Controller main(E_CONTROLLER_MASTER);

	while (true) {

// Tank Drive Control
		FL.move(main.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
		BL.move(main.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
		FR.move(main.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
		BR.move(main.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));

// Arm Control
		if(main.get_digital_new_press(DIGITAL_L1)) {
			if (armPos == 0) {
				targetArm = ARMMID;
				changeArmKp = ARMKP;
				armPos = 1;
			}
			else if (armPos == 1) {
				targetArm = ARMHIGH;
				changeArmKp = ARMKP;
				armPos = 2;
			}

		}
		if(main.get_digital_new_press(DIGITAL_L2)) {
			if (armPos == 2) {
				targetArm = ARMMID;
				changeArmKp = 0.3;
				armPos = 1;
			}
			else if (armPos == 1) {
				targetArm = ARMLOW;
				changeArmKp = 0.3;
				armPos = 0;
			}
		}

	// Claw Control
		if(main.get_digital_new_press(DIGITAL_R1)) {
			if (clawState == true) {
				targetClaw = CLAWUP;
				clawState = false;
			}
			else if (clawState == false) {
				targetClaw = CLAWDOWN;
				clawState = true;
			}
		}
		if(main.get_digital_new_press(DIGITAL_R2)) {
			if (liftDir == 0) {
				liftDir = 1;
			}
			else {
				liftDir = 0;
			}
		}
		if(main.get_digital(DIGITAL_R2)) {
			if (liftDir == -0) {
				LF.move(127);
			}
			else {
				LF.move(-127);
			}
		}
		else {
			LF.move(0);
			LF.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		}

		armControl(targetArm,changeArmKp);
		clawControl(targetClaw);
		printf("armPos: %.i\n",armPos);
		delay(5);
	}
}