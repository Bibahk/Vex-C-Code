#include "personal/main.h"
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "pros/rotation.hpp"
extern pros::Rotation lift_rotation;

bool SlowLift = false;
int liftSpeed = 12000;
const double maxLiftHeight = 5400;    // max height of lift
const double minLiftHeight = 100.0;   // minimum height of lift (to prevent wrapping back up)

void zero_lift() {
    lift.move_voltage(-6000); // Increased power to fight friction
    pros::delay(800);         // Given more time to reach the bottom     
    lift.move_voltage(0);
    lift.set_zero_position(0); 
}

void run_my_subsystems() {
    pros::Controller master_controller(pros::E_CONTROLLER_MASTER);

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
class LiftController {
private:
    pros::Motor& motor;
    lemlib::PID pid; 

public:
    // Constructor initializes the motor reference and LemLib PID gains
    LiftController(pros::Motor& liftMotor, float kP, float kI, float kD) 
        : motor(liftMotor), pid(kP, kI, kD, 15.0, true) {}

    // Preset height values (e.g., encoder degrees or ticks)
    enum class HeightPreset {
        DOWN = 0,
        LOW_GOAL = 250, // place holders 
        HIGH_GOAL = 600// place holders
    };

    void moveTo(float target, float timeoutMs = 2000) {
        pid.reset(); // Clear previous integral/derivative state
        uint32_t startTime = pros::millis();
while (pros::millis() - startTime < timeoutMs) {
            float currentPos = motor.get_position();
            float error = target - currentPos;

            // Target reach tolerance check
            if (std::abs(error) < 3.0) break;

            float output = pid.update(error);
            motor.move(std::clamp(output, -127.0f, 127.0f));

            pros::delay(10);
        }

        // Apply slight hold voltage or brake mode to counteract gravity
        motor.brake();
    }

    void moveToPreset(HeightPreset preset) {
        moveTo(static_cast<float>(preset));
    }
};
  // use these commands below to move the lift to a preset height in the auton      
//lift.moveToPreset(HeightPreset::HIGH_GOAL);
//lift.moveToPreset(HeightPreset::DOWN);


    //double currentHeight = lift.get_position();

    // Print to the controller screen (Row 2, Column 0)
    // Extra spaces at the end clear out any leftover characters from previous text
   // master_controller.print(2, 0, "Lift: %.1f    ", currentHeight);

   // if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        // go up
  //      if (currentHeight < maxLiftHeight) {
  //          lift.move_voltage(liftSpeed);
  //      } else {
   //         lift.move_voltage(0); // Stop at the top
    //    }
 //   } 
   // else if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        // go down
   //     if (currentHeight > minLiftHeight) {
   //         lift.move_voltage(-liftSpeed);
   //     } else {
   //         lift.move_voltage(0); // stops at bottom
   //     }
   // } 
  //  else {
  //      lift.move_voltage(0); // Holding position
   // }
}