#include "main.h"
#include "pros/motors.hpp"
#include <algorithm> 
#include <cmath>     
#include <stdlib.h>
#include <stdbool.h>

void run_my_subsystems();


const double WHEEL_SIZE_INCHES = 2.75;
const double MAX_MOTOR_RPM = 600.0;

//motor drfinitions

//motor definitions

pros::Motor small_left_motor(20); 
pros::Motor small_right_motor(11); // Note: Port 11 is also used in motors_right below

pros::Motor intake(1, pros::v5::MotorGears::blue); 

pros::MotorGroup lift({13, -15}, pros::v5::MotorGears::green);

pros::MotorGroup motors_left({-16, -17, -18}, pros::v5::MotorGears::blue);  
pros::MotorGroup motors_right({8, 9, 10}, pros::v5::MotorGears::blue);


void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}


void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
    pros::lcd::register_btn1_cb(on_center_button);

    // Force the motors to brake so turning is sharp
    motors_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    motors_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}


void disabled() {}
void competition_initialize() {}
void autonomous() {}

// The background task that handles the controller screen and rumble
void display_task_fn(void* param) {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    uint32_t startTime = pros::millis();
    
    bool showTemps = false; 
    bool show_11w = true;   
    bool btnPressed = false;
    int cycle_timer = 0;    

    while (true) {
        // up button toggle logic
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            if (!btnPressed) {
                showTemps = !showTemps; 
                master.clear();         
                btnPressed = true;
                cycle_timer = 0;        
            }
        } else {
            btnPressed = false;
        }

        // main screen, time and battery
        if (!showTemps) {
            if (cycle_timer == 5) {
                int total_seconds = (pros::millis() - startTime) / 1000;
                if (total_seconds < 60) {
                    master.print(0, 0, "Time: %d s       ", total_seconds);
                } else {
                    int minutes = total_seconds / 60;
                    int seconds = total_seconds % 60;
                    master.print(0, 0, "Time: %dm %02ds  ", minutes, seconds); 
                }
            } else if (cycle_timer == 10) {
                master.print(1, 0, "Battery: %.0f%%   ", pros::battery::get_capacity());
            }

            cycle_timer++;
            if (cycle_timer >= 50) cycle_timer = 0; 
        } 
        // temps screen
        else {
            if (show_11w) {
                if (cycle_timer == 5) { 
                    master.print(0, 0, "11W Temps       "); 
                } else if (cycle_timer == 10) { 
                    double lMax = std::max({motors_left.get_temperature(0), motors_left.get_temperature(1), motors_left.get_temperature(2)});
                    const char* lStatus = (lMax < 45) ? "Good" : (lMax < 55 ? "Med " : "Bad ");
                    master.print(1, 0, "L:%.0f%% %s       ", (lMax / 70.0 * 100.0), lStatus);
                } else if (cycle_timer == 15) { 
                    double rMax = std::max({motors_right.get_temperature(0), motors_right.get_temperature(1), motors_right.get_temperature(2)});
                    const char* rStatus = (rMax < 45) ? "Good" : (rMax < 55 ? "Med " : "Bad ");
                    master.print(2, 0, "R:%.0f%% %s       ", (rMax / 70.0 * 100.0), rStatus);
                }
            } else {
                if (cycle_timer == 5) {
                    master.print(0, 0, "5.5W Temps      ");
                } else if (cycle_timer == 10) {
                    double tL = small_left_motor.get_temperature();
                    const char* lStatus = (tL < 45) ? "Good" : (tL < 55 ? "Med " : "Bad ");
                    master.print(1, 0, "L:%.0f%% %s       ", (tL / 70.0 * 100.0), lStatus);
                } else if (cycle_timer == 15) {
                    double tR = small_right_motor.get_temperature();
                    const char* rStatus = (tR < 45) ? "Good" : (tR < 55 ? "Med " : "Bad ");
                    master.print(2, 0, "R:%.0f%% %s       ", (tR / 70.0 * 100.0), rStatus);
                }
            }

            cycle_timer++;
            if (cycle_timer >= 50) {
                cycle_timer = 0;
                show_11w = !show_11w; 
            }
        }

       
        // Get the highest temp in Celsius
        double absoluteMaxTemp = std::max({
            motors_left.get_temperature(0), motors_left.get_temperature(1), motors_left.get_temperature(2),
            motors_right.get_temperature(0), motors_right.get_temperature(1), motors_right.get_temperature(2),
            small_left_motor.get_temperature(), small_right_motor.get_temperature()
        });

        // convert to percentage
        double maxTempPercent = (absoluteMaxTemp / 70.0) * 100.0;

       // static int rumble_timer = 0;
        // Rumble now triggers when the hottest motor hits 85%
       // if (maxTempPercent >= 85) { 
     //       rumble_timer++;
      //      if (rumble_timer >= 150) {
      //          master.rumble("- . -"); 
       //         rumble_timer = 0;
       //     }
       // } else {
      //      rumble_timer = 0; 
       // }

        pros::delay(20);
    }
}


void opcontrol() {
   
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    // starts display task thingy
    pros::Task display_task(display_task_fn);

    auto applyExpo = [&](int input, int expoPercent) {
        int cubic = (input * input * input) / 10000;
        return ((100 - expoPercent) * input + expoPercent * cubic) / 100;
    };

    while (true) {
        int rawDrive = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rawTurn  = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        rawDrive = rawDrive * 100 / 127;
        rawTurn  = rawTurn  * 100 / 127;
        
        if (abs(rawDrive) < 4) rawDrive = 0;
        if (abs(rawTurn)  < 4) rawTurn  = 0;
        
        int drive = applyExpo(rawDrive, 35); // mabey change to 15 if srikar wants
        int turn  = rawTurn; 
        
        int leftPower  = drive + turn;
        int rightPower = drive - turn;
        
        const double MAX_WATTS = 3; 



        // change 40 to like 30 to make it slower ect
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            leftPower  = leftPower * 40 / 100;
            rightPower = rightPower * 40 / 100;  
        }  
     
    

        leftPower  = std::clamp(leftPower, -100, 100);
        rightPower = std::clamp(rightPower, -100, 100);
        
        if (abs(leftPower) < 5 && abs(rightPower) < 5) {
            motors_left.move_voltage(0);
            motors_right.move_voltage(0);
        } else {
            motors_left.move_voltage(leftPower * 120);
            motors_right.move_voltage(rightPower * 120);
        }
run_my_subsystems();

        pros::delay(20);    
    }
}