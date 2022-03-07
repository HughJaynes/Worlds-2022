#include "main.h"
#include "odometry.cpp"

void initialize() {
  // TASKS
  Task updateOdometryTask(baseOdometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "My Task");
  Task baseControlTask(baseControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "My Task");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}
