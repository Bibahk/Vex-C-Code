#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES


#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

extern pros::Motor small_left_motor;
extern pros::Motor small_right_motor;
extern pros::Motor intake;
extern pros::MotorGroup lift;
extern pros::MotorGroup motors_left;
extern pros::MotorGroup motors_right;

void on_center_button();
void display_task_fn(void* param);
void run_my_subsystems();
void zero_lift();
#endif

#endif  // _PROS_MAIN_H_