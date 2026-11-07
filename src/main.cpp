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
    lateral_controller(6, // proportional gain (kP)
                       0, // integral gain (kI)
                       4, // derivative gain (kD)
                       0, // anti windup
                       0, // small error range, in inches
                       0, // small error range timeout, in milliseconds
                       0, // large error range, in inches
                       0, // large error range timeout, in milliseconds
                       0  // maximum acceleration (slew)
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

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttle_curve(3,    // joystick deadband out of 127
                   10,   // minimum output where drivetrain will move out of 127
                   1.019 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steer_curve(3,    // joystick deadband out of 127
                10,   // minimum output where drivetrain will move out of 127
                1.019 // expo curve gain
    );

pros::MotorGroup lift_motors({15, -13}, pros::v5::MotorGears::blue);
pros::Motor toggle_motor(6, pros::v5::MotorGears::green);
pros::Distance clamp_detector(10);
pros::adi::Pneumatics clamp('A', false);

enum Auto { FourPin, Left, Skills, LQRight, RQLeft, None };

enum TeamColor { Red, Blue };

Auto SelectedAuto = Auto::LQRight;

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
  if (SelectedAuto == Auto::LQRight) {
    chassis.setPose(0, 0, 0);
    // Toggle
    toggle_motor.move_relative(-1400, 12000);
    pros::delay(2000);

    // Scoring first pin.
    lift_motors.move_relative(-700, 12000);
    chassis.moveToPoint(0, -15, 800, {.forwards = false});

    chassis.turnToPoint(-16, -15, 800, {.forwards = false});
    chassis.moveToPoint(-16, -15, 800, {.forwards = false});
    chassis.waitUntilDone();
    lift_motors.move_relative(600, 12000);
    pros::delay(500);
    clamp.extend();
    pros::delay(1000);

    // Getting next pin
    chassis.moveToPoint(0, -15, 800);
    lift_motors.move_relative(100, 12000);

    chassis.turnToPoint(-17.5, -33.5, 1000, {.forwards = false});
    chassis.moveToPoint(-10, -27.7, 1000, {.forwards = false});
    chassis.moveToPoint(-17.5, -33.5, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
  
    clamp.retract();
    pros::delay(500);

    lift.move_relative(-1200, 12000);

    chassis.moveToPoint(-23.8, -40, 800, {.forwards = false});
    chassis.turnToPoint(-23.8, -23, 00, {.forwards = false});
    chassis.moveToPoint(-23.8, -23, 800, {.forwards = false});
    chassis.waitUntilDone();
    // chassis.turnToPoint(16, -15, 800, {.forwards = false});
    // chassis.moveToPoint(16, -15, 800, {.forwards = false});
    lift_motors.move_relative(600, 12000);
    pros::delay(500);
    clamp.extend();
  } else if (SelectedAuto == Auto::RQLeft) {
    chassis.setPose(0, 0, 0);
    // Toggle
    toggle_motor.move_relative(-1300, 12000);
    pros::delay(2000);

    // Scoring first pin.
    lift_motors.move_relative(-700, 12000);
    chassis.moveToPoint(0, -15, 800, {.forwards = false});

    chassis.turnToPoint(16, -15, 800, {.forwards = false});
    chassis.moveToPoint(16, -15, 800, {.forwards = false});
    chassis.waitUntilDone();
    lift_motors.move_relative(600, 12000);
    pros::delay(500);
    clamp.extend();
    pros::delay(1000);

    // Getting next pin
    chassis.moveToPoint(0, -15, 800);
    lift_motors.move_relative(100, 12000);

    chassis.turnToPoint(17.5, -33.5, 1000, {.forwards = false});
    chassis.moveToPoint(10, -27.7, 1000, {.forwards = false});
    chassis.moveToPoint(17.5, -33.5, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
  
    clamp.retract();
    pros::delay(500);

    lift.move_relative(-1200, 12000);

    chassis.moveToPoint(23.8, -40, 800, {.forwards = false});
    chassis.turnToPoint(23.8, -23, 1000, {.forwards = false});
    chassis.moveToPoint(23.8, -23, 800, {.forwards = false});
    chassis.waitUntilDone();
    // chassis.turnToPoint(16, -15, 800, {.forwards = false});
    // chassis.moveToPoint(16, -15, 800, {.forwards = false});
    lift_motors.move_relative(600, 12000);
    pros::delay(500);
    clamp.extend();

    // lift.move_relative(600, 12000);
    // pros::delay(500);
    // clamp.extend();

  } else if (SelectedAuto == Auto::Left) {
  } else if (SelectedAuto == Auto::Skills) {
    chassis.setPose(0, 0, 0);
    toggle_motor.move_relative(-1300, 12000);
    pros::delay(2000);
    lift_motors.move_relative(-1700, 12000);
    chassis.moveToPoint(-0.5, -55, 4000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    lift_motors.move_relative(300, 12000);
    pros::delay(600);
    clamp.extend();

  } else if (SelectedAuto == Auto::FourPin) {
    // clamp extend is opening it
    chassis.setPose(69.38, -1.3, 270.0);
    toggle_motor.move_voltage(127);
    pros::delay(500);
    toggle_motor.move_voltage(0);
    // score matchload
    chassis.moveToPoint(45.5, -0.9, 1000);
    chassis.moveToPoint(45.88, 13.58, 10);
    lift_motors.move_relative(-1500, 12000);
    pros::delay(500);
    lift_motors.move_relative(1500, 12000);
    pros::delay(500);
    clamp.extend();
    // pick up 2nd pin
    chassis.moveToPoint(45.5, -0.9, 1000);
    chassis.moveToPoint(30.28, 17.05, 1000);
    clamp.retract();
    // clamp.extend();
    //  Go score pin #2
    lift_motors.move_relative(-1500, 12000);
    chassis.moveToPoint(37.28, -4.3, 1000);
    pros::delay(500);
    chassis.moveToPoint(46.77, -11.71, 1000);
    chassis.turnToHeading(0, 500);
    lift_motors.move_relative(1500, 12000);
    clamp.extend();
    // go pick up pin 3
    pros::delay(500);
    chassis.moveToPoint(32.38, -0.4, 1000);
    chassis.turnToHeading(90, 500);
    clamp.retract();

    // score pin #3
    chassis.moveToPoint(40.65, 15.22, 1000);
    lift_motors.move_relative(-1500, 12000);
    lift_motors.move_relative(1500, 12000);
    clamp.extend();
    // Go to pin #4
    // chassis.swingToPoint(90, 0, DriveSide::LEFT, 1000);
    // go pick up pin #4
    // clamp.retract();
    // score pin #4
    // lift_motors.move_relative(1500, 12000);
    // pros::delay(500);
    // chassis.moveToPoint(53.71,26.15,1000);
  }
}

// chassis.setPose(0, 0, 0);

// chassis.moveToPoint(0, 10, 1000);

// Scoring first pin

// chassis.moveToPoint(0, -15, 1000, {.forwards = false});

// chassis.turnToPoint(-11, -15, 3000, {.forwards = false});
// chassis.moveToPoint(-11, -15, 3000, {.forwards = false});

// chassis.moveToPoint(0, -15, 3000, {.forwards = true});

// // Getting next cup/pin

// chassis.turnToPoint(-21.5, -32, 3000, {.forwards = false});
// chassis.moveToPoint(-21.5, -32, 3000, {.forwards = false});

// chassis.moveToPoint(-29, -37.4, 1000, {.forwards = false});

// chassis.turnToPoint(-22.5, -13, 1000, {.forwards = false});
// chassis.moveToPoint(-22.5, -13, 1000, {.forwards = false});

// // Getting next cup/pin

// chassis.moveToPoint(-29, -37.4, 1000, {.forwards = true});

void opcontrol() {
  while (true) {
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    chassis.arcade(leftY, rightX);

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      toggle_motor.move(-127);
    } else {
      toggle_motor.move(0);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      lift_motors.move_voltage(-12000);
    }
    // L2 makes it go down (Safety checks commented out for tuning max height)
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      lift_motors.move_voltage(12000);
    } else {
      lift_motors.move_voltage(0);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      if (clamp.is_extended()) {
        clamp.retract();
      } else {
        clamp.extend();
      }
    }
  }
}