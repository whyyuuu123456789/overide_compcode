#include "main.h"

#include <algorithm>
#include <cmath>
#include <vector>

// ---------------------------------------------------------------------------
// Robot configuration
// Change these port numbers to match your robot.
// A negative port number reverses that motor.
// ---------------------------------------------------------------------------
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Drive base: 2 motors per side
pros::MotorGroup left_drive({-1, -2}, pros::MotorGears::blue);    // Left side, ports 1 & 2
pros::MotorGroup right_drive({3, 4}, pros::MotorGears::blue); // Right side, ports 3 & 4 (reversed)

// Lift
pros::MotorGroup lift({7, -8}, pros::MotorGears::blue);          // Lift, ports 7 & 8 (8 reversed)

// Joint: one 5.5W motor. 5.5W motors have no cartridge and always spin at 200 rpm (green).
pros::Motor joint(9, pros::MotorGears::blue);
pros::Motor claw(10, pros::MotorGears::green);                 // Claw, port 10, 5.5W motor (use -10 to reverse)

// Inertial sensor (IMU) - any free port works, just change the number
pros::Imu imu(20);



// Joystick values smaller than this are ignored, so stick drift doesn't move the robot
constexpr int DEADBAND = 5;

int apply_deadband(int value) {
	return (std::abs(value) < DEADBAND) ? 0 : value;
}


constexpr double WHEEL_DIAMETER = 3.25; 

constexpr double GEAR_RATIO = 0.75;

constexpr double PI = 3.14159265358979;
constexpr double INCHES_PER_DEGREE = (WHEEL_DIAMETER * PI * GEAR_RATIO) / 360.0;


// Joint power while R1 / R2 is held (0 to 127). Lower it if the joint moves too fast.
constexpr int JOINT_SPEED = 127;

// Claw power (0 to 127). X spins it at +CLAW_SPEED, Y at -CLAW_SPEED.
// To swap which button spins which way, change the claw's port to -10.
constexpr int CLAW_SPEED = 127;


struct PID {
	double kP, kI, kD;
	double integral_zone; // the I term only builds up when |error| is smaller than this

	double integral = 0;
	double prev_error = 0;

	PID(double p, double i, double d, double i_zone = 0)
	    : kP(p), kI(i), kD(d), integral_zone(i_zone) {}

	// Call before every new movement. Passing the first error stops a big D "kick" on the first loop.
	void reset(double start_error) {
		integral = 0;
		prev_error = start_error;
	}

	double update(double error) {
		if (std::fabs(error) < integral_zone) {
			integral += error;
		} else {
			integral = 0;
		}
		if (error * prev_error < 0) {
			integral = 0; // passed the target, so drop the built-up I to stop overshoot
		}

		double derivative = error - prev_error;
		prev_error = error;

		return kP * error + kI * integral + kD * derivative;
	}
};

// ---------------------------------------------------------------------------
// PID TUNING - start here when the robot overshoots, stops short, or wobbles
//
// How to tune (one controller at a time):
//   1. Set kI and kD to 0. Raise kP until the robot reaches the target and
//      oscillates a little around it.
//   2. Raise kD until the oscillation goes away.
//   3. If it stops a little short every time, add a small kI (try 0.05 - 0.2).
// ---------------------------------------------------------------------------
PID drive_pid(8.0, 0.0, 25.0, 3.0);   // error in inches  -> motor power
PID heading_pid(2.0, 0.0, 6.0);       // error in degrees -> correction that keeps drives straight
PID turn_pid(2.0, 0.0, 12.0, 15.0);   // error in degrees -> turning power

constexpr double DRIVE_TOLERANCE = 0.; // inches: close enough to count as "arrived"
constexpr double TURN_TOLERANCE  = 1.0; // degrees
constexpr int    SETTLE_TIME_MS  = 100; // must stay inside the tolerance this long to finish
constexpr int    LOOP_MS         = 10;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Wraps an angle to -180..180 so the robot always turns the short way
double wrap_180(double degrees) {
	return std::remainder(degrees, 360.0);
}

double average(const std::vector<double>& values) {
	if (values.empty()) return 0;
	double sum = 0;
	for (double v : values) sum += v;
	return sum / values.size();
}

// How far the robot has driven (in inches) based on all four drive encoders
double drive_position_inches() {
	double left  = average(left_drive.get_position_all());
	double right = average(right_drive.get_position_all());
	return ((left + right) / 2.0) * INCHES_PER_DEGREE;
}

// Sets drive power (-127 to 127), scaling both sides down together if one is too big,
// so the heading correction is never cut off
void set_drive(double left, double right) {
	double biggest = std::max(std::fabs(left), std::fabs(right));
	if (biggest > 127) {
		left  = left  * 127.0 / biggest;
		right = right * 127.0 / biggest;
	}
	left_drive.move(static_cast<int>(left));
	right_drive.move(static_cast<int>(right));
}

void stop_drive() {
	left_drive.brake();
	right_drive.brake();
}

// ---------------------------------------------------------------------------
// Position tracking (odometry) using the drive encoders + IMU
//
//   x: forward / back   (+x = forward from where the robot started)
//   y: left / right     (+y = right)
//   heading: degrees    (0 = facing +x, 90 = facing +y, i.e. turned right)
// ---------------------------------------------------------------------------
double robot_x = 0;
double robot_y = 0;
pros::Mutex pose_mutex;

void track_position() {
	double prev_inches = drive_position_inches();

	while (true) {
		double now_inches = drive_position_inches();
		double moved = now_inches - prev_inches;
		prev_inches = now_inches;

		double heading_rad = imu.get_heading() * PI / 180.0;

		pose_mutex.lock();
		robot_x += moved * std::cos(heading_rad);
		robot_y += moved * std::sin(heading_rad);
		double x = robot_x, y = robot_y;
		pose_mutex.unlock();

		pros::lcd::print(2, "x: %.1f  y: %.1f  h: %.1f", x, y, imu.get_heading());
		pros::delay(LOOP_MS);
	}
}

// Tell the robot where it is (call at the start of autonomous). heading must be 0 - 359.
void set_pose(double x, double y, double heading) {
	pose_mutex.lock();
	robot_x = x;
	robot_y = y;
	pose_mutex.unlock();
	imu.set_heading(heading);
}

// Drive straight forward (positive) or backward (negative) a number of inches.
// Uses the encoders for distance and the IMU to stay pointed straight.
void drive_distance(double inches, int max_speed = 127, int timeout_ms = 3000) {
	double start_inches   = drive_position_inches();
	double target_heading = imu.get_heading();

	drive_pid.reset(inches);
	heading_pid.reset(0);

	int settled_ms = 0;
	for (int elapsed = 0; elapsed < timeout_ms; elapsed += LOOP_MS) {
		double error         = inches - (drive_position_inches() - start_inches);
		double heading_error = wrap_180(target_heading - imu.get_heading());

		double power      = std::clamp(drive_pid.update(error), -(double)max_speed, (double)max_speed);
		double correction = heading_pid.update(heading_error);

		set_drive(power + correction, power - correction);

		settled_ms = (std::fabs(error) < DRIVE_TOLERANCE) ? settled_ms + LOOP_MS : 0;
		if (settled_ms >= SETTLE_TIME_MS) break;

		pros::delay(LOOP_MS);
	}
	stop_drive();
}

// Turn in place to face an absolute heading (0 = starting direction, 90 = right, -90 or 270 = left).
// Always takes the shortest way around.
void turn_to_heading(double target_heading, int max_speed = 127, int timeout_ms = 2000) {
	turn_pid.reset(wrap_180(target_heading - imu.get_heading()));

	int settled_ms = 0;
	for (int elapsed = 0; elapsed < timeout_ms; elapsed += LOOP_MS) {
		double error = wrap_180(target_heading - imu.get_heading());
		double power = std::clamp(turn_pid.update(error), -(double)max_speed, (double)max_speed);

		set_drive(power, -power); // positive power turns right (clockwise)

		settled_ms = (std::fabs(error) < TURN_TOLERANCE) ? settled_ms + LOOP_MS : 0;
		if (settled_ms >= SETTLE_TIME_MS) break;

		pros::delay(LOOP_MS);
	}
	stop_drive();
}

// Turn left (negative) or right (positive) by some number of degrees from where you are now
void turn_by(double degrees, int max_speed = 127, int timeout_ms = 2000) {
	turn_to_heading(imu.get_heading() + degrees, max_speed, timeout_ms);
}

// Turn to face the point (x, y), then drive to it.
// Set reverse = true to back into the point instead (the back of the robot faces it).
void move_to_point(double x, double y, bool reverse = false, int max_speed = 127) {
	pose_mutex.lock();
	double dx = x - robot_x;
	double dy = y - robot_y;
	pose_mutex.unlock();

	if (std::hypot(dx, dy) < DRIVE_TOLERANCE) return; // already there

	double angle = std::atan2(dy, dx) * 180.0 / PI;
	if (reverse) angle += 180.0;
	turn_to_heading(angle, max_speed);

	// Re-measure after the turn, since turning can nudge the robot a little
	pose_mutex.lock();
	double distance = std::hypot(x - robot_x, y - robot_y);
	pose_mutex.unlock();

	drive_distance(reverse ? -distance : distance, max_speed);
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
	joint.set_brake_mode(pros::MotorBrake::hold);
	claw.set_brake_mode(pros::MotorBrake::hold); // claw stays where it stopped

	// Encoders report in degrees (what drive_position_inches() expects)
	left_drive.set_encoder_units_all(pros::MotorUnits::degrees);
	right_drive.set_encoder_units_all(pros::MotorUnits::degrees);

	// Calibrate the IMU. KEEP THE ROBOT STILL for these ~2-3 seconds.
	pros::lcd::set_text(2, "Calibrating IMU...");
	imu.reset(true);

	// Start tracking x / y in the background
	pros::Task odom_task(track_position, "odometry");
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
void autonomous() {
	// Brake (instead of coast) so the robot stops exactly where PID tells it to
	left_drive.set_brake_mode_all(pros::MotorBrake::brake);
	right_drive.set_brake_mode_all(pros::MotorBrake::brake);

	set_pose(0, 0, 0); // robot starts at x = 0, y = 0, facing forward

	// Example route - replace with your own
	drive_distance(24);                
	turn_to_heading(90);               
	move_to_point(24, 24);             
	move_to_point(0, 0, true);         
	turn_to_heading(0);               

}

void opcontrol() {
	// Autonomous switches the drive to brake; go back to coast for driving
	left_drive.set_brake_mode_all(pros::MotorBrake::coast);
	right_drive.set_brake_mode_all(pros::MotorBrake::coast);

	int claw_direction = 0; // 1 = spinning from X, -1 = spinning from Y, 0 = stopped

	while (true) {
		//autonomous();

		int forward = apply_deadband(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		int turn    = apply_deadband(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));

		left_drive.move(forward + turn);
		right_drive.move(forward - turn);

		int lift_power = apply_deadband(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

		if (lift_power != 0) {
			lift.move(lift_power);
		} else {
			lift.brake(); 
		}

		// Joint: hold R2 to move it one way, hold R1 to move it the other way, let go and it holds still.
		// To swap the directions, change the joint's port to -9.
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			joint.move(JOINT_SPEED);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			joint.move(-JOINT_SPEED);
		} else {
			joint.brake();
		}

		// Claw: X toggles spinning one way, Y toggles spinning the other way.
		// Pressing the other button while it spins switches it straight to that direction.
		int claw_pressed = 0;
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) claw_pressed = 1;
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) claw_pressed = -1;

		if (claw_pressed != 0) {
			claw_direction = (claw_direction == claw_pressed) ? 0 : claw_pressed;
			if (claw_direction == 0) {
				claw.brake();
			} else {
				claw.move(claw_direction * CLAW_SPEED);
			}
		}


		pros::delay(10);
	}
}
