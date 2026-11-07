#include "personal/main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pid.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <algorithm>
#include <cmath>
#include <stdbool.h>
#include <stdlib.h>

const double MAX_MOTOR_RPM = 600.0;

// controller definition
pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor definitions

pros::MotorGroup lift({15, -13}, pros::v5::MotorGears::blue);

pros::MotorGroup motors_left({-3, -4, -5}, pros::v5::MotorGears::blue);
pros::MotorGroup motors_right({18, 19, 20}, pros::v5::MotorGears::blue);
// Limit switch plugged into ADI port 'a' (change letter if needed)

// piston defs
pros::adi::Pneumatics clamp('A', false);

// Encoders defs
pros::Rotation horizontal_encoder(-11); // Horizontal rotation sensor on port 11
pros::Rotation vertical_encoder(-12);   // Vertical rotation sensor on port 12

// Imu sensor definition
pros::Imu imu(7); // IMU on port 7

// drivetrain settings
lemlib::Drivetrain
    drivetrain(&motors_left,               // left motor group
               &motors_right,              // right motor group
               12.5,                       // 12.5 inch track width
               lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
               450,                        // drivetrain rpm is 450
               2                           // horizontal drift is 2 (for now)
    );

// lemlib odometry settings
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, 1.95, -3);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 1.95, 0);

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(6,  // proportional gain (kP)
                       0,  // integral gain (kI)
                       3, // derivative gain (kD)
                       0,  // anti windup
                       0,  // small error range, in inches
                       0,  // small error range timeout, in milliseconds
                       0,  // large error range, in inches
                       0,  // large error range timeout, in milliseconds
                       0   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2,  // proportional gain (kP) 2.4, Upper Limit (3.2)
                       0,  // integral gain (kI)
                       10, // derivative gain (kD) 20
                       0,  // anti windup
                       0,  // small error range, in inches
                       0,  // small error range timeout, in milliseconds
                       0,  // large error range, in inches
                       0,  // large error range timeout, in milliseconds
                       0   // maximum acceleration (slew)
    );

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");

  chassis.calibrate();
  pros::Task ControllerScreenTask([&]() {
    pros::delay(1000); // wait for imu to calibrate
    master.set_text(0, 0, "Robot Initializing");
    pros::delay(100);
    master.clear(); // clear controller screen
    pros::delay(200);

    pros::Task ControllerScreenTaks([]() {
      while (true) {
        master.print(0, 0, "Battery: %g %",
                     (std::to_string(pros::battery::get_capacity())).c_str());
        pros::delay(50);
        master.print(1, 0, "Timer: %.0f",
                     (std::to_string(pros::millis() / 1000.0) + "s").c_str());
        pros::delay(50);

        float Theta = chassis.getPose().theta;
        master.print(2, 0, "%.1f %.1f %.1f", chassis.getPose().x,
                     chassis.getPose().y, chassis.getPose().theta);
        pros::delay(50);
      }
    });
  });

  // Force the motors to brake so turning is sharp
  motors_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motors_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  lift.set_brake_mode(
      pros::E_MOTOR_BRAKE_HOLD); // Hold position when not moving
}

void disabled() {}
void competition_initialize() {}
void autonomous() {
  chassis.setPose(0, 0, 0);

  chassis.moveToPoint(0, 10, 2000);

  // Scoring first pin

  // chassis.moveToPoint(0, -15, 2000, {.forwards = false});

  // chassis.turnToPoint(-11, -15, 3000, {.forwards = false});
  // chassis.moveToPoint(-11, -15, 3000, {.forwards = false});

  // chassis.moveToPoint(0, -15, 3000, {.forwards = true});

  // // Getting next cup/pin

  // chassis.turnToPoint(-21.5, -32, 3000, {.forwards = false});
  // chassis.moveToPoint(-21.5, -32, 3000, {.forwards = false});

  // chassis.moveToPoint(-29, -37.4, 2000, {.forwards = false});

  // chassis.turnToPoint(-22.5, -13, 2000, {.forwards = false});
  // chassis.moveToPoint(-22.5, -13, 2000, {.forwards = false});

  // // Getting next cup/pin

  // chassis.moveToPoint(-29, -37.4, 2000, {.forwards = true});
}

void opcontrol() {
  // starts display task thingy

  auto applyExpo = [&](int input, int expoPercent) {
    int cubic = (input * input * input) / 10000;
    return ((100 - expoPercent) * input + expoPercent * cubic) / 100;
  };

  int rawDrive = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int rawTurn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  rawDrive = rawDrive * 100 / 127;
  rawTurn = rawTurn * 100 / 127;

  if (abs(rawDrive) < 4)
    rawDrive = 0;
  if (abs(rawTurn) < 4)
    rawTurn = 0;

  int drive = applyExpo(rawDrive, 35); // mabey change to 15 if srikar wants
  int turn = rawTurn;

  int leftPower = drive + turn;
  int rightPower = drive - turn;

  const double MAX_WATTS = 3;

  // change 40 to like 30 to make it slower ect
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    leftPower = leftPower * 40 / 100;
    rightPower = rightPower * 40 / 100;
  }

  leftPower = std::clamp(leftPower, -100, 100);
  rightPower = std::clamp(rightPower, -100, 100);

  if (abs(leftPower) < 5 && abs(rightPower) < 5) {
    motors_left.move_voltage(0);
    motors_right.move_voltage(0);
  } else {
    motors_left.move_voltage(leftPower * 120);
    motors_right.move_voltage(rightPower * 120);
  }
}