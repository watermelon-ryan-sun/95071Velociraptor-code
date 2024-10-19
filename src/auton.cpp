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
	turn(-60,3,1.0,0.5,1,1);
	pros::delay(50);
	move(42,0.1,0.25,0.2);
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
    clampRelease();
    pros::delay(50);
    turn(-109,3,1,0.5,1,1);
    pros::delay(50);
    moveBack(33.96,0.1,0.5,0.2);
    pros::delay(500);
    clampDown();
    pros::delay(50);
    turn(135,3,1,0.5,1,1);
    pros::delay(50);
    move(42,0.1,0.25,0.2);
	pros::delay(50);
	turn(180,3,1.0,0.5,9,9);
    pros::delay(50);
    move(48,0.1,0.25,0.2);
}
void skills(){
    moveBack(20,0.1,0.25,0.2);
    pros::delay(50);
    clampDown();
    pros::delay(500);
    RunIntake(23123213);
    pros::delay(50);
    turn(-90,3,0.9,0.3,1,1);
    pros::delay(50);
    move(50,0.1,0.25,0.2);
    pros::delay(50);
    moveBack(16,0.1,0.25,0.2);
    pros::delay(50);
    turn(-130,3,0.9,0.3,1,1);
    pros::delay(50);
    move(110,0.1,0.25,0.2);
    pros::delay(50);
    moveBack(140,0.1,0.25,0.2);
    pros::delay(50);
    clampRelease();
    pros::delay(50);
    move(16,0.1,0.25,0.2);
    pros::delay(50);
    turn(40,3,0.9,0.3,1,1);
    pros::delay(50);
    moveBack(42,0.1,0.25,0.2);
    pros::delay(500);
    clampDown();
}