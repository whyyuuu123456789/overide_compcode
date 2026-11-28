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

// Match loader piston (used by pid_straight). Change 'A' to the 3-wire port it's plugged into.
pros::adi::DigitalOut mlmech('A');

// Blue cartridge top speed. pid_straight power is a percent of this.
constexpr double DRIVE_MAX_RPM = 600.0;
// Same top speed in motor degrees per second (600 rpm * 360 deg / 60 s = 3600 deg/s)
constexpr double DRIVE_MAX_DEG_PER_SEC = DRIVE_MAX_RPM * 6.0;


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


constexpr int LOOP_MS = 10; // odometry loop speed

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

// Spins each side at a percent (-100 to 100) of top speed, like VEXcode's spin(..., percent)
void spin_drive_percent(double left_percent, double right_percent) {
	left_drive.move_velocity(static_cast<int>(left_percent / 100.0 * DRIVE_MAX_RPM));
	right_drive.move_velocity(static_cast<int>(right_percent / 100.0 * DRIVE_MAX_RPM));
}

// Sets each side's motor voltage as a percent (-100 to 100) of full power (12 V). Unlike
// spin_drive_percent, the motors don't run their own speed control on top of this, so what the
// PID asks for goes straight to the wheels. pid_turn uses this.
void spin_drive_voltage_percent(double left_percent, double right_percent) {
	left_drive.move_voltage(static_cast<int>(left_percent * 120.0)); // 100% = 12000 mV
	right_drive.move_voltage(static_cast<int>(right_percent * 120.0));
}

void stop_drive() {
	left_drive.brake();
	right_drive.brake();
}

double percent_to_deg_per_sec(double percent) {
	return percent / 100.0 * DRIVE_MAX_DEG_PER_SEC;
}

double deg_per_sec_to_percent(double deg_per_sec) {
	return deg_per_sec / DRIVE_MAX_DEG_PER_SEC * 100.0;
}


struct ProfilePoint {
	double pos; // where the robot should be
	double vel; // how fast it should be going (units per second)
};

struct MotionProfile {
	double sign;        // +1 forward / right, -1 backward / left
	double distance;    // total distance, always positive
	double accel;       // units per second per second
	double peak_vel;    // top speed actually reached (below max_vel on short moves)
	double accel_time;  // seconds spent speeding up (same time is spent slowing down)
	double cruise_time; // seconds spent at peak_vel
	double total_time;  // seconds for the whole move

	MotionProfile(double signed_distance, double max_vel, double max_accel) {
		sign = (signed_distance < 0) ? -1.0 : 1.0;
		distance = std::fabs(signed_distance);
		accel = max_accel;

		// Fastest speed reachable while still leaving room to stop
		peak_vel = std::min(max_vel, std::sqrt(distance * max_accel));
		if (peak_vel <= 0) { // zero-length move (or bad settings): nothing to plan
			accel_time = cruise_time = total_time = 0;
			return;
		}
		accel_time = peak_vel / accel;
		double accel_distance = 0.5 * accel * accel_time * accel_time;
		cruise_time = std::max(0.0, (distance - 2.0 * accel_distance) / peak_vel);
		total_time = 2.0 * accel_time + cruise_time;
	}

	// Target position and velocity `t` seconds after the move started
	ProfilePoint sample(double t) const {
		double pos, vel;
		if (t <= 0) {                                      // not started
			pos = 0;
			vel = 0;
		} else if (t < accel_time) {                       // speeding up
			pos = 0.5 * accel * t * t;
			vel = accel * t;
		} else if (t < accel_time + cruise_time) {         // cruising
			pos = 0.5 * accel * accel_time * accel_time + peak_vel * (t - accel_time);
			vel = peak_vel;
		} else if (t < total_time) {                       // slowing down
			double time_left = total_time - t;
			pos = distance - 0.5 * accel * time_left * time_left;
			vel = accel * time_left;
		} else {                                           // finished
			pos = distance;
			vel = 0;
		}
		return {pos * sign, vel * sign};
	}
};


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

void pid_turn(double delta) {
	double kp = 2.4;         // power (percent) per degree left to turn
	double ki = 0.04;        // builds extra power the longer the robot sits short of the target
	double kd = 16.0;        // brakes the turn: power per degree the error changed since last loop
	double i_zone = 15.0;    // degrees: ki only builds up once the robot is this close
	double max_power = 70.0; // percent of full power
	double tolerance = 1.0;  // degrees from the final angle that counts as "there"
	int settle_loops = 10;   // loops (10 ms each) the robot must stay inside tolerance to finish
	int timeout_ms = 2000;   // give up after this long
	bool print_debug = true; // true = print time, angle, error, power to the PROS terminal each loop

	double dest_angle = imu.get_rotation() + delta;
	double last_error = delta; // start equal to the first error so kd doesn't jump on the first loop
	double integral = 0;
	int settled = 0;
	double most_past = 0;      // farthest the IMU saw the robot go past the final angle (for debugging)
	uint32_t start_ms = pros::millis();

	while (settled < settle_loops && pros::millis() - start_ms < static_cast<uint32_t>(timeout_ms)) {
		double angle = imu.get_rotation();
		double error = dest_angle - angle; // degrees left to turn (+ = keep turning right)

		if (std::fabs(error) < i_zone) {
			integral += error;
		} else {
			integral = 0;
		}
		if (error * last_error < 0) {
			integral = 0; // just crossed the target, so drop the built-up I or it pushes the robot past
		}

		double derivative = error - last_error;
		last_error = error;

		double power = (error * kp) + (integral * ki) + (derivative * kd);
		power = std::clamp(power, -max_power, max_power);
		spin_drive_voltage_percent(power, -power);

		most_past = std::max(most_past, (delta < 0) ? error : -error);
		settled = (std::fabs(error) < tolerance) ? settled + 1 : 0;

		if (print_debug) {
			printf("%.3f, %.1f, %.1f, %.1f\n", (pros::millis() - start_ms) / 1000.0, angle, error, power);
		}
		pros::delay(10); // the IMU updates every 10 ms
	}
	stop_drive();

	if (print_debug) {
		const char* result = (settled >= settle_loops) ? "settled" : "TIMED OUT";
		double off = dest_angle - imu.get_rotation();
		printf("turn %s, %.1f degrees off, went %.1f past\n", result, off, most_past);
		// Same thing on the brain screen, so no cable is needed
		pros::lcd::print(3, "turn %s  off %.1f  past %.1f", result, off, most_past);
	}
}


void pid_straight(double distance, bool lowerml = false, double lowermlerror = 0) {
	double kp = 0.1;           // correction per motor degree the robot is behind / ahead of the profile
	double kd = 0.4;           // damps that correction
	double tolerance = 2.0;    // motor degrees from the end that counts as "there"
	double max_speed = 35.0;   // cruise speed, percent of top speed
	double max_accel = 90.0;  // speed ramp, percent per second (150 = 0 to 40% in about 0.27 s)
	double heading_kp = 1.0;   // steering, percent per degree the robot has turned off course
	double heading_tolerance = 1.0; // degrees off course that still counts as "straight"

	MotionProfile profile(distance, percent_to_deg_per_sec(max_speed), percent_to_deg_per_sec(max_accel));
	double timeout = profile.total_time + 1.0; // give up 1 s after the move should have finished

	double lastError = 0;
	int tolerance_count = 0;

	left_drive.tare_position_all();
	right_drive.tare_position_all();
	double start_heading = imu.get_rotation(); // hold this heading for the whole move
	uint32_t start_ms = pros::millis();

	while (tolerance_count < 5) {
		double t = (pros::millis() - start_ms) / 1000.0;
		if (t > timeout) break;

		ProfilePoint target = profile.sample(t);
		double lpos = average(left_drive.get_position_all());
		double rpos = average(right_drive.get_position_all());
		double pos = (lpos + rpos) / 2;

		double error = target.pos - pos; // + = behind the profile, - = ahead
		double derivative = error - lastError;
		lastError = error;

		// Feedforward (the profile's speed) does most of the work; PID fixes the rest
		double power = deg_per_sec_to_percent(target.vel) + (error * kp) + (derivative * kd);
		power = std::clamp(power, -100.0, 100.0);

		// Steer back to the starting heading. + = robot has turned left, so speed up the left
		// side and slow the right side (works the same driving forward or backward).
		double heading_error = start_heading - imu.get_rotation();
		double steer = heading_error * heading_kp;
		spin_drive_percent(power + steer, power - steer);

		// Only finished once the profile is done AND the robot is at the end AND pointing straight
		double remaining = distance - pos;
		if (t >= profile.total_time && std::fabs(remaining) < tolerance &&
		    std::fabs(heading_error) < heading_tolerance) {
			tolerance_count++;
		}
		else {
			tolerance_count = 0;
		}

		if (lowerml && std::fabs(remaining) < lowermlerror) {
			mlmech.set_value(true);
		}
		pros::delay(15);
	}
	stop_drive();
}

// Turn to face an absolute heading (0 = starting direction, 90 = right, -90 or 270 = left),
// taking the shortest way around.
void pid_turn_to(double target_heading) {
	pid_turn(wrap_180(target_heading - imu.get_heading()));
}

// Turn to face the point (x, y), then drive to it.
// Set reverse = true to back into the point instead (the back of the robot faces it).
void move_to_point(double x, double y, bool reverse = false) {
	pose_mutex.lock();
	double dx = x - robot_x;
	double dy = y - robot_y;
	pose_mutex.unlock();

	if (std::hypot(dx, dy) < 2.0 * INCHES_PER_DEGREE) return; // already there (same as pid_straight's tolerance)

	double angle = std::atan2(dy, dx) * 180.0 / PI;
	if (reverse) angle += 180.0;
	pid_turn_to(angle);

	// Re-measure after the turn, since turning can nudge the robot a little
	pose_mutex.lock();
	double distance = std::hypot(x - robot_x, y - robot_y);
	pose_mutex.unlock();

	pid_straight((reverse ? -distance : distance) / INCHES_PER_DEGREE);
}

// ---------------------------------------------------------------------------
// Autonomous selector
// Pick which autonomous runs with any of these:
//   - the LEFT / RIGHT buttons at the bottom of the brain screen
//   - tapping anywhere on the left or right half of the brain screen
//   - the LEFT / RIGHT arrow buttons on the controller
// The current pick shows on line 5 and on the controller screen. Defaults to RIGHT.
// ---------------------------------------------------------------------------
enum class Auton { LEFT, RIGHT };
Auton selected_auton = Auton::RIGHT;
bool auton_picked = false; // becomes true when LEFT or RIGHT is picked

void show_selected_auton() {
	const char* name = (selected_auton == Auton::LEFT) ? "LEFT " : "RIGHT";
	pros::lcd::print(5, "Auton: %s", name);
	master.print(0, 0, "Auton: %s", name);
}

void select_left_auton() {
	selected_auton = Auton::LEFT;
	auton_picked = true;
	show_selected_auton();
}

void select_right_auton() {
	selected_auton = Auton::RIGHT;
	auton_picked = true;
	show_selected_auton();
}

// Shows the selector and waits here until LEFT or RIGHT is picked (screen or controller).
void wait_for_auton_pick() {
	auton_picked = false;
	pros::lcd::set_text(4, "Pick auton: tap screen or ctrl < >");

	// Only count taps that start after this point, so a finger already on the screen doesn't pick
	int32_t last_press_count = pros::screen::touch_status().press_count;

	while (!auton_picked) {
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))  select_left_auton();
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) select_right_auton();

		// Reads the touchscreen directly, so this works even if the on-screen buttons miss the tap
		pros::screen_touch_status_s_t touch = pros::screen::touch_status();
		if (touch.touch_status != pros::E_TOUCH_ERROR && touch.press_count != last_press_count) {
			last_press_count = touch.press_count;
			if (touch.x < 240) { // screen is 480 px wide
				select_left_auton();
			} else {
				select_right_auton();
			}
		}

		pros::delay(20);
	}
	pros::lcd::clear_line(4);
}

void left_auton() {
	claw.set_brake_mode(pros::MotorBrake::hold);
	set_pose(0, 0, 0);
	claw.move(-CLAW_SPEED);
	pid_straight(3 / INCHES_PER_DEGREE);
	pid_straight(-4 / INCHES_PER_DEGREE);
	pid_straight(4 / INCHES_PER_DEGREE);
	pid_straight(-4 / INCHES_PER_DEGREE);
	pid_straight(16 / INCHES_PER_DEGREE);

	pid_turn(-90);

	joint.move_absolute(-800, 50);
	uint32_t joint_start_ms = pros::millis();
	while (std::fabs(joint.get_position() - 45) > 5 && pros::millis() - joint_start_ms < 1500) {
		pros::delay(10);
	}

	pid_straight(16.25/INCHES_PER_DEGREE);
	claw.brake();
	pros::delay(50);
	claw.move(CLAW_SPEED/5);
	pros::delay(900);
	claw.brake();
	pid_straight(-6.25/INCHES_PER_DEGREE);
	pid_turn(90);
	pid_straight(40/INCHES_PER_DEGREE);
	
}

void right_auton() {
	set_pose(0, 0, 0);

	claw.move(-CLAW_SPEED);
	pid_straight(3 / INCHES_PER_DEGREE);
	pid_straight(-4 / INCHES_PER_DEGREE);
	pid_straight(4 / INCHES_PER_DEGREE);
	pid_straight(-4 / INCHES_PER_DEGREE);
	pid_straight(16.25 / INCHES_PER_DEGREE);

	pid_turn(90);

	// move_absolute only sets a target and returns right away, so wait here until the joint
	// gets there (or 1.5 s passes). Velocity must be positive: the target decides the direction.
	joint.move_absolute(-750, 50);
	uint32_t joint_start_ms = pros::millis();
	while (std::fabs(joint.get_position() - 45) > 5 && pros::millis() - joint_start_ms < 1500) {
		pros::delay(10);
	}

	pid_straight(16.25/INCHES_PER_DEGREE);
	claw.brake();
	pros::delay(50);
	claw.move(CLAW_SPEED/5);
	pros::delay(900);
	claw.brake();
	pid_straight(-6.25/INCHES_PER_DEGREE);
	pid_turn(-90);
	pid_straight(40/INCHES_PER_DEGREE);
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

	// Auton selector: left screen button = LEFT, right screen button = RIGHT
	pros::lcd::register_btn0_cb(select_left_auton);
	pros::lcd::register_btn2_cb(select_right_auton);
	pros::lcd::set_text(7, "< LEFT               RIGHT >");
	show_selected_auton();

	// Drive coasts when sticks are released; lift holds its position
	left_drive.set_brake_mode_all(pros::MotorBrake::coast);
	right_drive.set_brake_mode_all(pros::MotorBrake::coast);
	lift.set_brake_mode_all(pros::MotorBrake::hold);
	joint.set_brake_mode(pros::MotorBrake::hold);
	joint.tare_position(); // joint's current spot = 0, so move_absolute targets are measured from here
	claw.set_brake_mode(pros::MotorBrake::hold); // claw stays where it stopped

	// Encoders report in degrees (what drive_position_inches() expects)
	left_drive.set_encoder_units_all(pros::MotorUnits::degrees);
	right_drive.set_encoder_units_all(pros::MotorUnits::degrees);

	// Calibrate the IMU. KEEP THE ROBOT STILL for these ~2-3 seconds.
	pros::lcd::set_text(2, "Calibrating IMU...");
	imu.reset(true);

	// Start tracking x / y in the background
	pros::Task odom_task(track_position, "odometry");

	// Pre-auton auton pick when NOT plugged into a field / competition switch.
	// When plugged in, competition_initialize() does the pick instead, so initialize()
	// never sits here waiting during a real match. Done after IMU calibration so tapping
	// the screen can't shake the robot while the IMU calibrates.
	if (!pros::competition::is_connected()) {
		wait_for_auton_pick();
	}
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
void competition_initialize() {
	// Pre-auton auton pick on a field / competition switch. If nobody picks before the
	// match starts, this gets stopped automatically and RIGHT runs.
	wait_for_auton_pick();
}

/**
 * Runs the user autonomous code.
 */
void autonomous() {
	// Brake (instead of coast) so the robot stops exactly where PID tells it to
	left_drive.set_brake_mode_all(pros::MotorBrake::brake);
	right_drive.set_brake_mode_all(pros::MotorBrake::brake);
	joint.set_brake_mode(pros::MotorBrake::hold);

	if (selected_auton == Auton::LEFT) {
		left_auton();
	} else {
		right_auton();
	}
}

void opcontrol() {
	int claw_direction = 0; // 1 = spinning from X, -1 = spinning from Y, 0 = stopped

	// TEMP test: runs once when driver control starts. Must stay OUTSIDE the while loop,
	// or it re-runs every loop and the robot moves in tiny start/stop steps.
	// Uses brake mode like autonomous, so it stops the same way it will in a match.
	// Skipped when plugged into a field / competition switch, so it can't run in a real match.
	if (!pros::competition::is_connected()) {
		left_drive.set_brake_mode_all(pros::MotorBrake::brake);
		right_drive.set_brake_mode_all(pros::MotorBrake::brake);
		joint.set_brake_mode(pros::MotorBrake::hold);

		// The auton was already picked in pre-auton (end of initialize)
		pros::delay(1000);     // 1 s to get your hand off the robot before it moves
		autonomous();          // run the picked autonomous once at the start of driver control

		// Autonomous switches the drive to brake; go back to coast for driving
		left_drive.set_brake_mode_all(pros::MotorBrake::coast);
		right_drive.set_brake_mode_all(pros::MotorBrake::coast);
	}

	while (true) {
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
