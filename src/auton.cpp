#include "PIDControls.h"
#include "Functions.h"
#include "main.h"
#include "MotorInit.h"
void blueLeftAWP(){
    Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(40.87,0.2,0.3,0.2);
	pros::delay(50);
	clampDown();
	pros::delay(1000);
	RunIntake(212);
	pros::delay(50);
	turn(60,3,0.8,0.3,1,1);
	pros::delay(50);
	move(40,0.1,0.25,0.2);
	pros::delay(50);
	turn(-154,3,0.8,0.3,9,9);
	pros::delay(50);
	move(74,0.2,0.5,0.2);
    pros::delay(50);
    intake.move_velocity(0);
	pros::delay(50);
    intake.tare_position();
    pros::delay(50);
    intake.move_absolute(500,600);
    pros::delay(50);
	turn(-116,3,0.8,0.3,1,1);
	pros::delay(50);
	move(27,0.2,1,0.2);
}
void redRightAWP(){
    Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(42,0.2,0.3,0.2);
	pros::delay(300);
	clampDown();
	pros::delay(1000);
	RunIntake(212);
	pros::delay(50);
	turn(-60,3,0.8,0.3,1,1);
	pros::delay(50);
	move(40,0.1,0.25,0.2);
	pros::delay(50);
	turn(154,3,0.8,0.3,9,9);
	pros::delay(50);
	move(73,0.2,0.5,0.2);
    pros::delay(300);
    intake.move_velocity(0);
    pros::delay(50);
	turn(116,3,0.8,0.3,1,1);
	pros::delay(50);
	move(27,0.2,1,0.2);
}
void blueRightAWP(){
    Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(40.87,0.2,0.3,0.2);
	pros::delay(50);
	clampDown();
	pros::delay(500);
	RunIntake(212);
	pros::delay(50);
	turn(-60,3,1.0,0.5,1,1);
	pros::delay(50);
	move(44,0.1,0.2,0.2);
	pros::delay(50);
	turn(154,3,1.0,0.5,9,9);
	pros::delay(50);
	move(75,0.2,0.5,0.2);
    pros::delay(50);
    intake.move_velocity(0);
	pros::delay(50);
    intake.tare_position();
    pros::delay(50);
    intake.move_absolute(500,600);
    pros::delay(50);
	turn(116,3,1,0.5,1,1);
	pros::delay(50);
	move(27,0.2,1,0.2);
}
void RedLeftAWP(){
    Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(40.87,0.2,0.3,0.2);
	pros::delay(50);
	clampDown();
	pros::delay(500);
	RunIntake(212);
	pros::delay(50);
	turn(60,3,1.0,0.5,1,1);
	pros::delay(50);
	move(42,0.1,0.25,0.2);
	pros::delay(50);
	turn(-154,3,1.0,0.5,9,9);
	pros::delay(50);
	move(75,0.2,0.5,0.2);
    pros::delay(50);
    intake.move_velocity(0);
    pros::delay(50);
	turn(-116,3,1,0.5,1,1);
	pros::delay(50);
	move(27,0.2,1,0.2);
}
void blueSRightAWP(){
    Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(40.87,0.2,0.3,0.2);
	pros::delay(50);
	clampDown();
	pros::delay(500);
	RunIntake(212);
	pros::delay(50);
	turn(60,3,1.0,0.3,1,1);
	pros::delay(50);
	move(42,0.1,0.25,0.2);
	pros::delay(50);
	turn(26,3,1.0,0.3,9,9);
	pros::delay(50);
	moveBack(77,0.2,0.5,0.2);
    pros::delay(50);
    intake.move_velocity(0);
	pros::delay(50);
    intake.tare_position();
    pros::delay(50);
    intake.move_absolute(500,600);
    pros::delay(50);
    clampRelease();
    pros::delay(50);
    turn(-80,3,1,0.3,1,1);
    pros::delay(50);
    moveBack(36,0.1,0.5,0.2);
    pros::delay(500);
    clampDown();
    pros::delay(50);
    turn(-135,3,1,0.5,1,1);
    pros::delay(50);
    move(42,0.1,0.25,0.2);
	pros::delay(50);
	turn(180,3,1.0,0.3,9,9);
    pros::delay(50);
    move(48,0.1,0.25,0.2);
}
void skills(){
	IMU.set_rotation(0);
	pros::lcd::print(0,"IMU degrees at the start %f",IMU.get_rotation());
    intake.move_velocity(600);
    pros::delay(150);
	move(4,0.2,0.3,0.2);
	pros::delay(50);
    turn(-118,2.5,0.2,0.01,1,1);
	pros::lcd::print(1,"rotated after first turn %f", IMU.get_rotation());
    pros::delay(50);
    moveBack(40,0.2,0.3,0.2);
	pros::delay(450);
	clampDown();
	pros::delay(50);
	turn(0,2.5,0.2,0.01,1,1);
	pros::lcd::print(2,"IMU degrees after %f",IMU.get_rotation());
	pros::delay(100);
	move(36,0.2,0.3,0.2);
	double lastPosition = intake.get_position(); 
	if(intake.get_position() < lastPosition+600){
		intake.move_velocity(-200);
		pros::delay(50);
		intake.move_velocity(600);
	}
	pros::delay(100);
	turn(90,2.5,0.2,0.01,1,1);
	pros::lcd::print(0,"IMU degrees %f",IMU.get_heading());
	pros::delay(100);
	move(44,0.2,0.3,0.2);
	lastPosition = intake.get_position(); 
	pros::delay(100);
	turn(25,2.5,0.2,0.01,1,1);
	if(intake.get_position() < lastPosition+100){
		intake.move_velocity(-50);
		pros::delay(50);
		intake.move_velocity(600);
	}
	pros::delay(100);
	pros::lcd::print(0,"IMU degrees %f",IMU.get_heading());
	move(39,0.2,0.3,0.2);
	if(intake.get_position() < lastPosition+100){
		intake.move_velocity(-200);
		pros::delay(50);
		intake.move_velocity(600);
	}
	pros::delay(100);
	moveBack(38,0.2,0.3,0.2);
	lastPosition = intake.get_position(); 
	if(intake.get_position() < lastPosition+100){
		intake.move_velocity(-200);
		pros::delay(50);
		intake.move_velocity(600);
	}
	pros::delay(50);
	turn(177,2.5,0.2,0.001,1,1);
	pros::delay(100);
	move(55,0.2,0.1,0.2);
	if(intake.get_position() < lastPosition+600){
		intake.move_velocity(-200);
		pros::delay(50);
		intake.move_velocity(600);
	}
	pros::delay(2000);
	turn(45,2.5,0.2,0.01,1,1);
	pros::delay(50);
	move(20,0.2,0.3,0.2);
	lastPosition = intake.get_position(); 
	if(intake.get_position() < lastPosition+600){
		intake.move_velocity(-200);
		pros::delay(50);
		intake.move_velocity(600);
	}
	pros::delay(2000);
	turn(-20,2.5,0.2,0.01,1,1);
	pros::delay(50);
	moveBack(30,0.2,0.3,0.2);
	pros::delay(50);
	clampRelease();
	pros::delay(50);
	intake.move_absolute(-6,50);
	pros::delay(50);
	move(10,0.2,0.3,0.2);
	pros::delay(50);
	turn(88.5,2.5,0.2,0.01,1,1);
	pros::delay(50);
	moveBack(48,0.1,0.7,0.5);
	pros::delay(1500);
	clampDown();
	pros::delay(50);
	intake.move_velocity(500);
	pros::delay(50);
	turn(0,2.5,0.2,0.01,1,1);
	pros::delay(100);
	move(40,0.2,0.3,0.2);
	pros::delay(100);
	turn(-90,2.5,0.2,0.01,1,1);
	pros::lcd::print(0,"IMU degrees %f",IMU.get_heading());
	pros::delay(100);
	move(44,0.2,0.3,0.2);
	pros::delay(100);
	turn(-25,2.5,0.2,0.01,1,1);
	pros::delay(100);
	pros::lcd::print(0,"IMU degrees %f",IMU.get_heading());
	move(48,0.2,0.3,0.2);
	pros::delay(100);
	moveBack(40,0.2,0.3,0.2);
	pros::delay(50);
	turn(180,2.5,0.2,0.01,1,1);
	pros::delay(100);
	move(53,0.2,0.25,0.2);
	pros::delay(500);
	turn(-45,2.5,0.2,0.01,1,1);
	pros::delay(50);
	move(12,0.2,0.3,0.2);
	pros::delay(50);
	turn(-30,2,1,0.5,1,1);
	pros::delay(50);
	moveBack(10,0.2,0.3,0.2);
	pros::delay(50);
	clampRelease();
	pros::delay(50);
	move(12,0.2,0.3,0.2);
	pros::delay(50);
	turn(45,2,1,0.5,1,1);
}