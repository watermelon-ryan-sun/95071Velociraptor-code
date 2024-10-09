#include "PIDControls.h"
#include "Functions.h"
#include "main.h"
#include "MotorInit.h"
void rightSideAwp(){
    /*moveBack(48, 0.1,0.2,0.1);
    pros::delay(200);
    clampDown();
    pros::delay(200);
    turn(-90);
    pros::delay(200);
    */
   Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(37.84,0.1,0.3,0.2);
	pros::delay(50);
	clampDown();
	pros::delay(50);
	RunIntake(212);
	pros::delay(50);
	turn(-45,0.9,25,0.05,9,9);
	pros::delay(50);
	move(35,0.1,0.25,0.2);
	//pros::delay(1425);
	pros::lcd::print(0, "Deg %f", IMU.get_rotation());
	//ringInArm();
	pros::delay(50);
	turn(-122.5,5,150,0.05,9,9);
	pros::delay(50);
	moveBack(36,0.1,0.3,0.2);
}
void skills(){

}