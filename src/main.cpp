#include "main.h"

int lPos = 0;
int rState = 0;
bool frontPos = true;

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

    ADIDigitalOut LC (LCPISTON);
    ADIDigitalOut T1 (T1PISTON);
    ADIDigitalOut T2 (T2PISTON);
    ADIDigitalOut TC (TCPISTON);

    Imu IMU (IMUPORT);

    Controller master (E_CONTROLLER_MASTER);

    Task liftControlTask (liftControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Lift Control");
    Task baseOdometryTask (baseOdometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");
    Task baseControlTask (baseControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Base Control");
    Task getBearingTask (getBearing, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors");
    Task backIntakeTask (toggleTilter, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Back Intake");
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
  moveBase(30,3000);
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

    ADIDigitalOut LC (LCPISTON);

    Controller master (E_CONTROLLER_MASTER);

	while (true)
    {
        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
            if (lPos <= LIFTDOWN) {
                lPos = LIFTMID;
            } else if (lPos == LIFTMID) {
                lPos = LIFTUP;
            }
        } else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) {
            if (lPos == LIFTUP) {
                lPos = LIFTMID;
            } else if (lPos == LIFTMID) {
                rState = 0;
                RI.move(0);
                lPos = LIFTDOWN;
            }
        }

        moveLift(lPos);

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)) {
            // lcToggle();
            frontPos = !frontPos;
            if(frontPos){
              LC.set_value(LOW);
            }else {
              LC.set_value(HIGH);
            }
        }

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
          toggleSwitch();
        }

        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
            if (rState == 0 && lPos > LIFTDOWN) {
                rState = 1;
                RI.move(RINGSPEED);
            } else {
                rState = 0;
                RI.move(0);
            }
        }

        if (master.get_digital(E_CONTROLLER_DIGITAL_B)) {
            if (rState == 1) {
                rState = 2;
                RI.move(-RINGSPEED);
            }
        } else if (!master.get_digital(E_CONTROLLER_DIGITAL_B)) {
            if (rState == 2) {
                rState = 1;
                RI.move(RINGSPEED);
            }
        }

        // Tank Drive controls
      	FL.move(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
      	ML.move(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
      	BL.move(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));

      	FR.move(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
      	MR.move(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
      	BR.move(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));

        printf("Heading: %.2f\n",bearing);
      	delay(5);
    }
}
