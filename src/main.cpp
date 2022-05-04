#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    Motor FL (FLPORT, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
    Motor FR (FRPORT, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
    Motor ML (MLPORT, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
    Motor MR (MRPORT, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
    Motor BL (BLPORT, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
    Motor BR (BRPORT, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
    Motor LI (LIPORT, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);
    Motor RI (RIPORT, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);

    Controller master (E_CONTROLLER_MASTER);
    Task baseController (baseControl, (void*)"ignore", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Controls the drivetrain");
    Task subsystemController (subsystemControl, (void*)"ignore", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Controls Pneumatics, Lift and Rings");
    
    Imu IMU(IMUPORT);
    IMU.reset(); 
    LI.tare_position();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
    changeTilter();
    delay(3000);
    moveBase(-600, -600, 3000, 0.3, 0.3, 80);
    delay(3000);
    changeTilter();
    changeLiftUp();
    changeLiftUp();
    delay(200);
    changeRingOnOff();
    delay(1000);
    rotateBase(-65, 2000);
    delay(2000);
    moveBase(600, 600, 6000, 0.08, 0.25, 70);
    delay(6000);
    changeRingOnOff();
    /*
    delay(2000);
    changeLiftDown();
    changeLiftDown();
    changeClamp();
    moveBase(-300,-300,2000);
    delay(2000);
    changeTilter();
    delay(250);
    rotateBase(135,2000);
    */
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
    Motor FL (FLPORT);
    Motor FR (FRPORT);
    Motor ML (MLPORT);
    Motor MR (MRPORT);
    Motor BL (BLPORT);
    Motor BR (BRPORT);
    Motor LI (LIPORT);
    Motor RI (RIPORT);

    Controller master (E_CONTROLLER_MASTER);

	while (true)
    {
        // Subsystem controls
        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
            changeLiftUp();
        } else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) {
            changeLiftDown();
        }

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)) {
            changeClamp();
        }

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
            changeTilter();
        }

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
            changeRingOnOff();
        }

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
            changeRingUpDown();
        }

        // Tank-Drive controls
      	FL.move(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
      	ML.move(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
      	BL.move(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
        
      	FR.move(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
      	MR.move(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
      	BR.move(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));

      	delay(5);
    }
}