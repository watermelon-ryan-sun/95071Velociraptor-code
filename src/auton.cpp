#include "PIDControls.h"
#include "Functions.h"
#include "main.h"
#include "MotorInit.h"
void BlueLeftAwp(){
    Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(40.87,0.2,0.3,0.2);
	pros::delay(1000);
	clampDown();
	pros::delay(50);
	RunIntake(212);
	pros::delay(50);
	turn(60,3,1.0,0.5,1,1);
	pros::delay(50);
	move(42,0.1,0.25,0.2);
	pros::delay(50);
	turn(-154,3,1.0,0.5,9,9);
	pros::delay(50);
	move(51.94,0.2,0.5,0.2);
	pros::delay(50);
	turn(116,3,1,0.5,1,1);
	pros::delay(50);
	move(48,0.2,0.25,0.2);
}
void redRightAwp(){
    Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(40.87,0.2,0.3,0.2);
	pros::delay(1000);
	clampDown();
	pros::delay(50);
	RunIntake(212);
	pros::delay(50);
	turn(-60,3,1.0,0.5,1,1);
	pros::delay(50);
	move(42,0.1,0.25,0.2);
	pros::delay(50);
	turn(154,3,1.0,0.5,9,9);
	pros::delay(50);
	move(51.94,0.2,0.5,0.2);
	pros::delay(50);
	turn(116,3,1,0.5,1,1);
	pros::delay(50);
	move(48,0.2,0.25,0.2);
}
void blueRightAWP(){
    moveBack(36,0.2,0.3,0.2);
    pros::delay(100);
    clampDown();
    pros::delay(50);
    RunIntake(12312);
    pros::delay(50);
    turn(-135,3,1,0.5,9,9);
    pros::delay(50);
    move(33.94,0.1,0.25,0.2);
    pros::delay(100);
    moveBack(67.8,0.1,0.25,0.2);
    turn(45,3,1,0.5,1,1);
    move(24,0.1,0.25,0.2);
}
void RedLeftAWP(){
    moveBack(36,0.2,0.3,0.2);
    pros::delay(100);
    clampDown();
    pros::delay(50);
    RunIntake(12312);
    pros::delay(50);
    turn(135,3,1,0.5,9,9);
    pros::delay(50);
    move(33.94,0.1,0.25,0.2);
    pros::delay(100);
    moveBack(67.8,0.1,0.25,0.2);
    turn(-45,3,1,0.5,1,1);
    move(24,0.1,0.25,0.2);
}
void blueSRightAWP(){

}
void skills(){

}