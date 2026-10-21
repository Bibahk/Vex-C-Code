#include "personal/main.h"
#include "lemlib/api.hpp"
#include <cmath>
#include <algorithm>
#include "pros/llemu.hpp" 

// Hardware declarations
extern pros::MotorGroup lift;
extern pros::Motor intake;
extern pros::adi::Pneumatics clamp;
extern pros::adi::Pneumatics rotate;
extern pros::adi::Button bumper_switch; // Updated to bumper switch
extern pros::Rotation lift_rotation;

int liftSpeed = 12000;
double currentHeight = 0;
const float MAX_HEIGHT = 250.0; // Safety ceiling to stop lift from breaking at top

// Lift controller class using PID
class LiftController {
private:
    pros::MotorGroup& motorGroup;
    lemlib::PID pid;

public:
    LiftController(pros::MotorGroup& liftGroup, float kP, float kI, float kD) 
        : motorGroup(liftGroup), pid(kP, kI, kD, 15.0, true) {}

    enum class HeightPreset { 
        DOWN = 0, 
        LOW_GOAL = 200, // change to wtv height it is
        HIGH_GOAL = 250 // change to wtv height it is
    };

    void moveTo(float target, float timeoutMs = 2000) {
        pid.reset();
        uint32_t startTime = pros::millis();

        while (pros::millis() - startTime < timeoutMs) {
            float currentPos = lift_rotation.get_position() / 100.0;
            float error = target - currentPos;

            if (std::abs(error) < 3.0) break;

            float output = pid.update(error);

            // Safety check: Prevent lift from breaking itself at physical maximums (Max commented out for tuning, bottom active)
            // if (currentPos >= MAX_HEIGHT && output > 0) output = 0;
            
            // Stop downward movement if bumper is hit or height is 0 or below 
             if ((bumper_switch.get_value() == 0 || currentPos <= 0.0) && output < 0) output = 0;

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

// Helper function to zero the lift on startup
void zero_lift() {
    lift.move_voltage(-6000);
    pros::delay(800);
    lift.move_voltage(0);
    lift.set_zero_position(0);
}

// Driver control loop
void run_my_subsystems() {
    static pros::Controller master_controller(pros::E_CONTROLLER_MASTER);

    // Intake control (L1 / L2)
    if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        intake.move_voltage(-12000);
    } else if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        intake.move_voltage(12000);
    } else {
        intake.move_voltage(0);
    }

    // Speed toggle (X button)
    if (master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        if (liftSpeed == 12000) {
            liftSpeed = 6000;
        } else {
            liftSpeed = 12000;
        }
    }

    // Read lift height from rotation sensor (convert centidegrees to degrees)
    currentHeight = lift_rotation.get_position() / 100.0;

    // Print height to V5 Brain screen
    pros::lcd::print(1, "Lift Pos: %.1f deg", currentHeight);

    // Throttled controller screen print to prevent lag
    static uint32_t print_timer = 0;
    if (pros::millis() - print_timer > 250) {
        master_controller.print(2, 0, "Lift: %.1f deg ", currentHeight);
        print_timer = pros::millis();
    }

    // Independent Bumper Switch Check using get_new_press()
    if (bumper_switch.get_new_press()) {
        lift_rotation.set_position(0);
        currentHeight = 0.0;
    }

    // MAKE SURE TO DELETE AFTER TUNING PID, OR ELSE ROBOT WONT MOVE
    if (master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        liftController.moveToPreset(LiftController::HeightPreset::HIGH_GOAL);
    } else if (master_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        liftController.moveToPreset(LiftController::HeightPreset::DOWN);
    }

    // R1 makes it go UP
    if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        lift.move_voltage(-liftSpeed);
    }
    // R2 makes it go DOWN (Safety checks commented out for tuning)
    else if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        // if (bumper_switch.get_value() != 0 && currentHeight > 0.0) {
            lift.move_voltage(liftSpeed);
        // } else {
        //     lift.move_voltage(0); // Safety stop so it won't go below 0
        // }
    } else {
        lift.move_voltage(0);
    }

    // Pneumatics Logic
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