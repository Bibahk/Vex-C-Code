#include "personal/main.h"
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "pros/rotation.hpp"

// Declare your rotation sensor externally so it's accessible here
extern pros::Rotation lift_rotation;

int liftSpeed = 12000;
const double maxLiftHeight = 5400.0;    // max height of lift
const double minLiftHeight = 100.0;   // minimum height of lift

// ==================== LIFT CONTROLLER CLASS ====================
class LiftController {
private:
    pros::Motor& motor;
    lemlib::PID pid; 

public:
    LiftController(pros::Motor& liftMotor, float kP, float kI, float kD) 
        : motor(liftMotor), pid(1.0, 0.0, 0.0, 15.0, true) {}

    enum class HeightPreset {
        DOWN = 0,
        LOW_GOAL = 250,   
        HIGH_GOAL = 600   
    };

    void moveTo(float target, float timeoutMs = 2000) {
        pid.reset(); 
        uint32_t startTime = pros::millis();
        
        while (pros::millis() - startTime < timeoutMs) {
            float currentPos = motor.get_position();
            float error = target - currentPos;

            if (std::abs(error) < 3.0) break;

            float output = pid.update(error);
            motor.move(std::clamp(output, -127.0f, 127.0f));

            pros::delay(10);
        }
        motor.brake();
    }

    void moveToPreset(HeightPreset preset) {
        moveTo(static_cast<float>(preset));
    }
};

// helper functoins
void zero_lift() {
    lift.move_voltage(-6000); 
    pros::delay(800);         
    lift.move_voltage(0);
    lift.set_zero_position(0); 
}

// driver control 
void run_my_subsystems() {
    static pros::Controller master_controller(pros::E_CONTROLLER_MASTER);

    //intake control
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
    double currentHeight = lift_rotation.get_position() / 100.0;

    // Throttled screen print to prevent controller lag
    static uint32_t print_timer = 0;
    if (pros::millis() - print_timer > 250) {
        master_controller.print(2, 0, "Lift: %.1f    ", currentHeight);
        print_timer = pros::millis();
    }

    if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        // Go up
        if (currentHeight < maxLiftHeight) {
            lift.move_voltage(liftSpeed);
        } else {
            lift.move_voltage(0); 
        }
    } 
    else if (master_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        // Go down
        if (currentHeight > minLiftHeight) {
            lift.move_voltage(-liftSpeed);
        } else {
            lift.move_voltage(0); 
        }
    } 
    else {
        lift.move_voltage(0); 
    }

    // ----------------- PNEUMATICS (A & B) -----------------
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