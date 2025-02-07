#include "main.h"
#include "Functions.h"
#include "PIDControls.h"
#include "MotorInit.h"
#include "auton.h"
#include "odom.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);	
	//autonSelector();
	pros::Task OdomCalib(recordPosition);
	XPos = 0;
	YPos = 0;
	avgThetaForArc = 0;
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
	Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	//autonSelector();
}

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
void autonomous() {
	//pros::Task position_task(intake_fn, (void *)"PROS", TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT, "Print X and Y Task");
	/*switch(autonSelected){
		case 0:
			RedLeftAWP();
			break;
		case 1: 
			blueLeftAWP();
			break;
		case 2: 
			redRightAWP();
			break;
		case 3: 
			blueRightAWP();
			break;
		case 4:
			skills();
			break;
}*/
//blueRightAWP();
specialOdomSkills();
//skills2();
//move(60,0.2,0.3,0.2);
//skills();
//specialOdomSkills();
}

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
	Odometry.set_position(0);
	specialOdomSkills();
	while(true){
		driveFunc(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
		pros::lcd::print(0,"XPos %f", XPos);
	pros::lcd::print(1,"YPos %f", YPos);
	pros::lcd::print(2, "currentAngle %f", IMU.get_heading());
	pros::lcd::print(3,"BM%d, RM%f", Odometry.get_position(), RM_MOTOR.get_position());
	pros::lcd::print(4,"avgThetaForArc:%f", avgThetaForArc);
	//pros::lcd::print(5,"XGlobal %f", deltaXGlobal);
	pros::delay(500);
	}
}
