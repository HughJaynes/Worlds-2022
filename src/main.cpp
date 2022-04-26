#include "main.h"

int armPos = 0;

void initialize() {
	// Tasks to run
	Task updateOdometryTask(baseOdometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "My Task");
	Task baseControlTask(baseControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "My Task");
	Task liftControlTask(liftControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "My Task");

	ADIDigitalOut CL (1);
	ADIDigitalOut TC (2);
	ADIDigitalOut LT (3);
	ADIDigitalOut RT (4);

	Motor FL (FLPORT,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
	Motor FR (FRPORT,E_MOTOR_GEARSET_18,true,E_MOTOR_ENCODER_DEGREES);
	Motor ML (MLPORT,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
	Motor MR (MRPORT,E_MOTOR_GEARSET_18,true,E_MOTOR_ENCODER_DEGREES);
	Motor BL (BLPORT,E_MOTOR_GEARSET_18,true,E_MOTOR_ENCODER_DEGREES);
	Motor BR (BRPORT,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
	Motor LF (LFPORT,E_MOTOR_GEARSET_36,true,E_MOTOR_ENCODER_DEGREES);
	Motor RM (RMPORT,E_MOTOR_GEARSET_06,true,E_MOTOR_ENCODER_DEGREES);
}


void disabled() {

}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	Motor FL (FLPORT,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
	Motor FR (FRPORT,E_MOTOR_GEARSET_18,true,E_MOTOR_ENCODER_DEGREES);
	Motor ML (MLPORT,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
	Motor MR (MRPORT,E_MOTOR_GEARSET_18,true,E_MOTOR_ENCODER_DEGREES);
	Motor BL (BLPORT,E_MOTOR_GEARSET_18,true,E_MOTOR_ENCODER_DEGREES);
	Motor BR (BRPORT,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
	Motor LF (LFPORT,E_MOTOR_GEARSET_36,true,E_MOTOR_ENCODER_DEGREES);
	Motor RM (RMPORT,E_MOTOR_GEARSET_06,false,E_MOTOR_ENCODER_DEGREES);

	ADIDigitalOut CL (CLPORT);
	ADIDigitalOut TC (TCPORT);
	ADIDigitalOut LT (LTPORT);
	ADIDigitalOut RT (RTPORT);

	Controller main (E_CONTROLLER_MASTER);

	while (true) {

		double leftPower = main.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) * -1;
		double rightPower = main.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y) * -1;

		FL.move(leftPower);
		ML.move(leftPower);
		BL.move(leftPower);
		FR.move(rightPower);
		MR.move(rightPower);
		BR.move(rightPower);

		if (main.get_digital_new_press(DIGITAL_L1)) {
			if (armPos < 3) {
				armPos += 1;
			}
		}
		if (main.get_digital_new_press(DIGITAL_L2)) {
			if (armPos > 1) {
				armPos -= 1;
			}
		}

		if (armPos == 0) {
			moveLift(LFPOS0);
		}
		if (armPos == 1) {
			moveLift(LFPOS1);
		}
		if (armPos == 2) {
			moveLift(LFPOS2);
		}
		if (armPos == 3) {
			moveLift(LFPOS3);
		}

		if(main.get_digital_new_press(DIGITAL_R1)) {
			toggleClaw();
		}

		if(main.get_digital_new_press(DIGITAL_R2)) {
			toggleTilterClaw();
			delay(750);
			toggleTilter();
		}
		if (main.get_digital_new_press(DIGITAL_X)) {
			toggleRings();
		}

		delay(5);
	}
}
