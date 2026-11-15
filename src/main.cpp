#include "main.h"

// ---------------------------------------------------------------------------
// Robot configuration
// Change these port numbers to match your robot.
// A negative port number reverses that motor.
// ---------------------------------------------------------------------------
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Drive base: 2 motors per side
pros::MotorGroup left_drive({1, 2}, pros::MotorGears::blue);    // Left side, ports 1 & 2
pros::MotorGroup right_drive({-3, -4}, pros::MotorGears::blue); // Right side, ports 3 & 4 (reversed)

// Lift
pros::MotorGroup lift({5, -6}, pros::MotorGears::red);          // Lift, ports 5 & 6 (6 reversed)

// Joystick values smaller than this are ignored, so stick drift doesn't move the robot
constexpr int DEADBAND = 5;

int apply_deadband(int value) {
	return (std::abs(value) < DEADBAND) ? 0 : value;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Override Comp Code");

	// Drive coasts when sticks are released; lift holds its position
	left_drive.set_brake_mode_all(pros::MotorBrake::coast);
	right_drive.set_brake_mode_all(pros::MotorBrake::coast);
	lift.set_brake_mode_all(pros::MotorBrake::hold);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code.
 */
void autonomous() {}

/**
 * Runs the operator control code.
 *
 * Controls:
 *   Left stick up/down    -> drive forward/backward
 *   Left stick left/right -> turn
 *   Right stick up/down   -> lift up/down
 */
void opcontrol() {
	while (true) {
		// Left-stick arcade drive
		int forward = apply_deadband(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		int turn    = apply_deadband(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));

		left_drive.move(forward + turn);
		right_drive.move(forward - turn);

		// Lift on right stick
		int lift_power = apply_deadband(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

		if (lift_power != 0) {
			lift.move(lift_power);
		} else {
			lift.brake(); // Hold the lift in place when the stick is released
		}

		pros::delay(20); // Run every 20 ms
	}
}
