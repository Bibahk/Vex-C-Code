#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/distance.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({8, -9, -10},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-3, 4, 5}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

//Sensors
pros::Optical myOptical(19); // Optical sensor on port 19
pros::Distance myDistance(7); // Distance sensor on port 7

// Inertial Sensor on port 20
pros::Imu imu(20);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
//pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(2);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.5, // 10.5 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 600
                              8 // horizontal drift is 8. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(12.5, // proportional gain (kP) make it the highest possible value untill it starts to oscillate, then back it down a bit
                                            0, // integral gain (kI)
                                            38, // derivative gain (kD) if kp is occilating make kd value higher and keep on repeating untill kd cant fix it
                                             0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2.35, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             20, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() { // this code runs first when the robot is turned on
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {  
        while (true) {    // all of this code just shows the robot's position on the brain SCREEN and logs it to the console
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    chassis.setPose({0, 0, 0}); // set the initial pose to (0, 0, 0)
chassis.moveToPoint(0, 29.5, 1500, {} ,false); // move to the point (0, 29.5) with a maximum speed of 64 (out of 127) and a timeout of 1500ms
pros::delay(2000); // wait for 2 seconds	
chassis.moveToPoint(15.5,61.5,1500,{.forwards= false,.maxSpeed = 64});

// turntoheading
//swingtoheading - usefull for tight corners
chassis.turnToHeading(90,750); // theta is the angle in degrees, timeout is in milliseconds
chassis.swingToHeading(45, lemlib::DriveSide::RIGHT, 750); // theta is the angle in degrees, timeout is in milliseconds

chassis.turnToHeading(35,750,{.maxSpeed = 64}); // theta is the angle in degrees, timeout is in milliseconds

chassis.moveToPoint(10, 50, 1000, {.maxSpeed = 64}); // move to the point (15.5, 29.5) with a maximum speed of 64 (out of 127) and a timeout of 1500ms
chassis.turnToHeading(225, 500,{.maxSpeed = 64});
chassis.turnToHeading(405,500,{.maxSpeed = 64});
chassis.swingToHeading(455,lemlib::DriveSide::RIGHT,500,{.maxSpeed = 64});

chassis.moveToPoint(22.5,53.5,750,{.maxSpeed = 64},false); // move to the point (15.5, 29.5) with a maximum speed of 64 (out of 127) and a timeout of 1500ms
intakeMotor.move(-127); // run the intake motor at full speed
pros::delay(500);
intakeMotor.move(0); // sto p the intake motor

chassis.moveToPoint(22.5, 0, 2500); // move to the point (15.5, 29.5) with a maximum speed of 64 (out of 127) and a timeout of 1500ms
}

/** d
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
if (myOptical.get_hue() > 190 && myOptical.get_hue() < 220) { // if the optical sensor detects a hue between 190 and 220 (which is the range for blue)
// if blue run this code
pros::delay(500); // wait for 500 milliseconds
myPiston.extend(); // extend the piston
}else if (myOptical.get_hue() > 5 && myOptical.get_hue() < 30) { // if the optical sensor detects a hue between 0 and 10 (which is the range for red)

	pros::delay(500); // wait for 500 milliseconds
	intakeMotor.move(-127); // extakes the object at full speed
}else{
		myPiston.retract(); // if the optical sensor does not detect blue or red, retract the piston
		intakeMotor.move(127);  
	}
if(myDistance.get() < 200){ 
	myPiston.extend(); 
} else{
	myPiston.retract();
}
        // delay to save resources
        pros::delay(10);
    }
}