#include "personal/main.h"
#include "pros/misc.h"

bool SlowLift = false;
int liftSpeed = 12000;

void zero_lift() {
    lift.move_voltage(-6000); // Increased power to fight friction
    pros::delay(800);         // Given more time to reach the bottom     
    lift.move_voltage(0);
    lift.set_zero_position(0); 
}

void run_my_subsystems() {
    pros::Controller master_controller(pros::E_CONTROLLER_MASTER);
    
    // Set brake mode once
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // intake
    if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        intake.move_voltage(-12000); 
    } 
    else if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        intake.move_voltage(12000); 
    } 
    else {
        intake.move_voltage(0); 
    }

    // press and hold x to make it slower
    if (master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        if (liftSpeed == 12000) {
            liftSpeed = 6000;
        } else {
            liftSpeed = 12000;
        }
    }

    double maxLiftHeight = 5400;    // max height of lift
    double minLiftHeight = 100.0;   // minimum height of lift (to prevent wrapping back up)
    double currentHeight = lift.get_position();

    // Print to the controller screen (Row 2, Column 0)
    // Extra spaces at the end clear out any leftover characters from previous text
    master_controller.print(2, 0, "Lift: %.1f    ", currentHeight);

    if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        // go up
        if (currentHeight < maxLiftHeight) {
            lift.move_voltage(liftSpeed);
        } else {
            lift.move_voltage(0); // Stop at the top
        }
    } 
    else if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        // go down
        if (currentHeight > minLiftHeight) {
            lift.move_voltage(-liftSpeed);
        } else {
            lift.move_voltage(0); // stops at bottom
        }
    } 
    else {
        lift.move_voltage(0); // Holding position
    }
}