#include "personal/main.h"
#include "pros/apix.h"
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "pros/rotation.hpp"
#include <cmath>
#include <algorithm>

// Declare your rotation sensor externally so it's accessible here
extern pros::Rotation lift_rotation;
extern pros::MotorGroup lift;
extern pros::Motor intake;
extern pros::adi::Pneumatics clamp;
extern pros::adi::Pneumatics rotate;

int liftSpeed = 12000;
double currentHeight = 0;

// lift controller class
class LiftController {
private:
    pros::MotorGroup& motorGroup;
    lemlib::PID pid; 

public:
    LiftController(pros::MotorGroup& liftGroup, float kP, float kI, float kD) 
        : motorGroup(liftGroup), pid(kP, kI, kD, 15.0, true) {}

    enum class HeightPreset {
        DOWN = 0,
        LOW_GOAL = 250,   
        HIGH_GOAL = 200   
    };

    void moveTo(float target, float timeoutMs = 2000) {
        pid.reset(); 
        uint32_t startTime = pros::millis();
        
        while (pros::millis() - startTime < timeoutMs) {
            float currentPos = lift_rotation.get_position() / 100.0;
            float error = target - currentPos;

            if (std::abs(error) < 3.0) break;

            float output = pid.update(error);
            motorGroup.move(std::clamp(output, -127.0f, 127.0f));

            pros::delay(10);
        }
        motorGroup.brake();
    }

    void moveToPreset(HeightPreset preset) {
        moveTo(static_cast<float>(preset));
    }
};

// Instantiate the lift controller
LiftController liftController(lift, 1.0, 0.0, 0.0);

// helper functions
void zero_lift() {
    lift.move_voltage(-6000); 
    pros::delay(800);        
    lift.move_voltage(0);
    lift.set_zero_position(0); 
}

// driver control 
void run_my_subsystems() {
    static pros::Controller master_controller(pros::E_CONTROLLER_MASTER);

    // intake control
    if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        intake.move_voltage(-12000); 
    } 
    else if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        intake.move_voltage(12000); 
    } 
    else {
        intake.move_voltage(0); 
    }
    
    if (master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        if (liftSpeed == 12000) {
            liftSpeed = 6000;
        } else {
            liftSpeed = 12000;
        }
    }

    // Read from the rotation sensor (divided by 100.0 to convert centidegrees to degrees)
    currentHeight = lift_rotation.get_position() / 100.0;
    // find raw position
    // Print to the V5 Brain screen
    pros::lcd::print(1, "Lift Pos: %.1f deg", currentHeight);

    // Throttled controller screen print to prevent lag (prints with "deg")
    static uint32_t print_timer = 0;
    if (pros::millis() - print_timer > 250) {
        master_controller.print(2, 0, "Lift: %.1f deg   ", currentHeight);
        print_timer = pros::millis();
    }

    // D-Pad buttons for PID 
    if (master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        liftController.moveToPreset(LiftController::HeightPreset::HIGH_GOAL);
    }
    else if (master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        liftController.moveToPreset(LiftController::HeightPreset::DOWN);
    }
    // Direct manual control on R1 and R2 
    else if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        lift.move_voltage(liftSpeed); // Go up
    } 
    else if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        lift.move_voltage(-liftSpeed); // Go down
    } 
    else {
        lift.move_voltage(0); 
    }   

    // Pneumatics logic 
    if (master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        if (clamp.is_extended()) {
            clamp.retract();
        } else {
            clamp.extend();
        }
    }

    if (master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        if (rotate.is_extended()) {
            rotate.retract();
        } else {
            rotate.extend();
        }
    } 
}