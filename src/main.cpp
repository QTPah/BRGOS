#include "main.h"

// ======== Config ========

// Motor ports
constexpr int LEFT_MOTOR_PORT = 1;
constexpr int RIGHT_MOTOR_PORT = 2;
constexpr int SHOULDER_MOTOR1_PORT = 3;
constexpr int SHOULDER_MOTOR2_PORT = 4;
constexpr int ELBOW_MOTOR_PORT = 5;

// Piston ports
constexpr char INNER_PISTON_PORT = 'A';
constexpr char OUTER_PISTON_PORT = 'B';

// Movement
constexpr int REGULAR_DRIVETRAIN_SPEED = 70; // Drivetrain speed for regular movement in percent
constexpr int SLOW_DRIVETRAIN_SPEED = 30; // Drivetrain speed for slow movement in percent

constexpr int REGULAR_TURN_SPEED = 70; // Drivetrain turn speed for regular movement in percent
constexpr int SLOW_TURN_SPEED = 30; // Drivetrain turn speed for slow movement in percent


// Arm 
constexpr int SHOULDER_SPEED = 65; // Shoulder motor speed in percent
constexpr int ELBOW_SPEED = 100; // Elbow motor speed in percent

constexpr int MIN_SHOULDER_POSITION = 0; // Minimum shoulder position in degrees
constexpr int MAX_SHOULDER_POSITION = 100; // Maximum shoulder position in degrees

constexpr int MIN_ELBOW_POSITION = 0; // Minimum elbow position in degrees
constexpr int MAX_ELBOW_POSITION = 100; // Maximum elbow position in degrees


// ========================

void initialize() {
	pros::lcd::initialize();
  pros::lcd::set_background_color(255, 0, 0);
	pros::lcd::set_text(1, "Loading BRGOS...");
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
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	pros::Motor left_m(LEFT_MOTOR_PORT * -1);
	pros::Motor right_m(RIGHT_MOTOR_PORT);

  pros::MotorGroup shoulder({
      SHOULDER_MOTOR1_PORT * -1,
      SHOULDER_MOTOR2_PORT
  });

  pros::Motor elbow(ELBOW_MOTOR_PORT);

  pros::ADIDigitalOut inner_piston(INNER_PISTON_PORT, false);
  pros::ADIDigitalOut outer_piston(OUTER_PISTON_PORT, false);

  std::uint32_t shoulder_hold = 0, elbow_hold = 0;

  std::bool percision_mode = false;
  std::bool quick_mode = false;


	while (true) {

    // ======== Drivetrain control ========
		int dir = master.get_analog(ANALOG_LEFT_Y) * (percision_mode ? SLOW_DRIVETRAIN_SPEED :
                                                  quick_mode ? 100 : REGULAR_DRIVETRAIN_SPEED)
                                                  / 100;
		int turn = master.get_analog(ANALOG_LEFT_X) * (percision_mode ? SLOW_TURN_SPEED :
                                                  quick_mode ? (100 / (dir/2)) : REGULAR_TURN_SPEED) // Turn speed is slower in quick mode when driving fast
                                                  / 100;
		left_m.move(dir - turn);
		right_m.move(dir + turn);
    // ===================================


    // Using seperate if statements for simultaneous button presses

    if (master.get_digital(DIGITAL_L1)) {
      inner_piston.set_value(!inner_piston.get_value());      // Toggle inner inner_piston
    } 

    if (master.get_digital(DIGITAL_L2)) {
      outer_piston.set_value(!outer_piston.get_value());       // Toggle outer inner_piston
    }

   
    // ======== Manual arm control ========
    
    std::bool arm_moved = false;

    if (master.get_analog(ANALOG_RIGHT_Y) != 0 && master.get_digital(DIGITAL_R1)) {

      // Limit shoulder Movement
      if (
          (master.get_analog(ANALOG_RIGHT_Y) > 0 && shoulder.get_position() < MAX_SHOULDER_POSITION) ||
          (master.get_analog(ANALOG_RIGHT_Y) < 0 && shoulder.get_position() > MIN_SHOULDER_POSITION)
      ) {
        shoulder.move(master.get_analog(ANALOG_RIGHT_Y) * SHOULDER_SPEED / 100);
        shoulder_hold = shoulder.get_position();
        arm_moved = true;
      }

    } else if (master.get_analog(ANALOG_RIGHT_Y) != 0 && master.get_digital(DIGITAL_R2)) {

      // Limit elbow Movement
      if (
          (master.get_analog(ANALOG_RIGHT_Y) > 0 && elbow.get_position() < MAX_ELBOW_POSITION) ||
          (master.get_analog(ANALOG_RIGHT_Y) < 0 && elbow.get_position() > MIN_ELBOW_POSITION)
      ) {
        elbow.move(master.get_analog(ANALOG_RIGHT_Y) * ELBOW_SPEED / 100);
        elbow_hold = elbow.get_position();
        arm_moved = true;
      }

    }

    if (!arm_moved) {
      // Hold arm position
      shoulder.move_absolute(shoulder_hold, 127);
      elbow.move_absolute(elbow_hold, 127);

    }

    // ===================================


    // TODO: Add button to toggle percision and quick mode,
    // add free movement mode for arm to tare position,
    // add movement presets for; preparing to climb, climbing, preparing to pick up ring, prepare to place ring, prepare to sweep ring, attack mode etc.


		pros::delay(20);
	}
}
