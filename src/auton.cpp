#include "PIDControls.h"
#include "Functions.h"
#include "main.h"
#include "MotorInit.h"
#include "odom.h"
void blueRightAWP(){
	IMU.set_rotation(0);
    Arm.tare_position();
	intake.tare_position();
	Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(46,0.2,0.3,0.2);
	pros::delay(250);
	clampDown();
	pros::delay(500);
	RunIntake(212);
	pros::delay(50);
	turn(-90,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(37,0.2,0.3,0.2);
	pros::delay(50);
	turn(-175,2.5,0.2,0.1,9,9);
	pros::delay(50);
	move(24,0.2,0.3,0.2);
    pros::delay(350);
	moveBack(30,0.2,0.3,0.2);
	turn(90,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(45,0.2,0.5,0.2);
}
void blueRightSpecial(){
	IMU.set_rotation(0);
    Arm.tare_position();
	intake.tare_position();
	Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(46,0.2,0.3,0.2);
	pros::delay(250);
	clampDown();
	pros::delay(500);
	RunIntake(212);
	pros::delay(50);
	turn(-90,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(37,0.2,0.3,0.2);
	pros::delay(50);
	turn(-175,2.5,0.2,0.1,9,9);
	pros::delay(50);
	move(24,0.2,0.3,0.2);
    pros::delay(250);
	moveBack(30,0.2,0.3,0.2);
	pros::delay(50);
	turn(-120,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBack(90,0.2,0.5,0.2);
}
void RedLeftSpecial(){
	IMU.set_rotation(0);
    Arm.tare_position();
	intake.tare_position();
	Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(46,0.2,0.3,0.2);
	pros::delay(250);
	clampDown();
	pros::delay(500);
	RunIntake(212);
	pros::delay(50);
	turn(90,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(37,0.2,0.3,0.2);
	pros::delay(50);
	turn(175,2.5,0.2,0.1,9,9);
	pros::delay(50);
	move(24,0.2,0.3,0.2);
    pros::delay(250);
	moveBack(30,0.2,0.3,0.2);
	intake.move_velocity(0);
	pros::delay(50);
	turn(120,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBack(90,0.2,0.5,0.2);
}
void redRightAWP(){
	IMU.set_rotation(0);
    Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(46,0.2,0.3,0.2);
	pros::delay(300);
	clampDown();
	pros::delay(1000);
	RunIntake(212);
	pros::delay(50);
	turn(-90,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(35,0.1,0.25,0.2);
	pros::delay(50);
	turn(90,2.5,0.2,0.1,9,9);
	move(50,0.2,0.3,0.2);
	pros::delay(50);
	intake.move_velocity(0);
}
void blueLeftAWP(){
	IMU.set_rotation(0);
    Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(46,0.2,0.3,0.2);
	pros::delay(300);
	clampDown();
	pros::delay(1000);
	RunIntake(212);
	pros::delay(50);
	turn(90,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(35,0.1,0.25,0.2);
	pros::delay(250);
	turn(-90,2.5,0.2,0.1,9,9);
	move(50,0.2,0.3,0.2);
	pros::delay(50);
	intake.move_velocity(0);
}
void RedLeftAWP(){
	IMU.set_rotation(0);
    Arm.tare_position();
	intake.tare_position();
	Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(46,0.2,0.3,0.2);
	pros::delay(250);
	clampDown();
	pros::delay(500);
	RunIntake(212);
	pros::delay(50);
	turn(90,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(37,0.2,0.3,0.2);
	pros::delay(50);
	turn(175,2.5,0.2,0.1,9,9);
	pros::delay(50);
	move(24,0.2,0.3,0.2);
    pros::delay(350);
	moveBack(30,0.2,0.3,0.2);
	turn(-90,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(45,0.2,0.5,0.2);
}
void RedSLeftAWP(){
	IMU.set_rotation(0);
    Arm.tare_position();
	intake.tare_position();
	Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(46,0.2,0.3,0.2);
	pros::delay(250);
	clampDown();
	pros::delay(250);
	RunIntake(212);
	pros::delay(50);
	turn(90,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(38,0.2,0.3,0.2);
	pros::delay(50);
	turn(175,2.5,0.2,0.1,9,9);
	pros::delay(50);
	move(24,0.2,0.3,0.2);
    pros::delay(250);
	moveBack(40,0.2,0.3,0.2);
	pros::delay(10);
	turn(-75,2.5,0.2,0.1,1,1);
	pros::delay(10);
	clampRelease();
	pros::delay(10);
	move(80,0.2,0.3,0.2);
	pros::delay(10);
	turn(40,2.5,0.2,0.1,1,1);
	pros::delay(10);
	moveBack(51,0.2,0.3,0.2);
	pros::delay(250);
	clampDown();
	pros::delay(50);
	turn(-90,2.5,0.2,0.1,1,1);
	pros::delay(10);
	move(30,0.2,0.3,0.2);
	pros::delay(50);
	turn(90,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(30,1,1,1);
	//intake.move_velocity(0);
	
}
void blueSRightAWP(){
	IMU.set_rotation(0);
    Arm.tare_position();
	intake.tare_position();
	Arm.tare_position();
	intake.tare_position();
	bool mode = false;
	bool mode2 = true;
	bool mode3 = true;
	moveBack(46,0.2,0.3,0.2);
	pros::delay(250);
	clampDown();
	pros::delay(250);
	RunIntake(212);
	pros::delay(50);
	turn(-90,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(38,0.2,0.3,0.2);
	pros::delay(50);
	turn(-175,2.5,0.2,0.1,9,9);
	pros::delay(50);
	move(24,0.2,0.3,0.2);
    pros::delay(250);
	moveBack(40,0.2,0.3,0.2);
	pros::delay(10);
	turn(75,2.5,0.2,0.1,1,1);
	pros::delay(10);
	clampRelease();
	pros::delay(10);
	move(80,0.2,0.3,0.2);
	pros::delay(10);
	turn(-40,2.5,0.2,0.1,1,1);
	pros::delay(10);
	moveBack(51,0.2,0.3,0.2);
	pros::delay(250);
	clampDown();
	pros::delay(50);
	turn(90,2.5,0.2,0.1,1,1);
	pros::delay(10);
	move(30,0.2,0.3,0.2);
	pros::delay(50);
	turn(-90,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(30,1,1,1);
	//intake.move_velocity(0);
	
}
void skills(){
	IMU.set_rotation(0);
	pros::lcd::print(0,"IMU degrees at the start %f",IMU.get_rotation());
    intake.move_velocity(450);
    pros::delay(500);
	move(5,0.2,0.3,0.2);
	pros::delay(50);
    turn(-100,2.5,0.15,0.1,1,1);
	pros::lcd::print(1,"rotated after first turn %f", IMU.get_rotation());
    pros::delay(50);
    moveBack(43,0.2,0.3,0.2);
	pros::delay(450);
	clampDown();
	pros::delay(50);
	turn(0,2.5,0.15,0.1,1,1);
	pros::lcd::print(2,"IMU degrees after %f",IMU.get_rotation());
	pros::delay(50);
	move(45,0.2,0.3,0.2);
	double lastPosition = intake.get_position(); 
	pros::delay(50);
	turn(88,2.5,0.1,0.1,1,1);
	pros::lcd::print(0,"IMU degrees %f",IMU.get_heading());
	pros::delay(50);
	move(44,0.2,0.3,0.2);
	lastPosition = intake.get_position(); 
	pros::delay(50);
	turn(27,2.5,0.2,0.1,1,1);
	pros::delay(50);
	pros::lcd::print(0,"IMU degrees %f",IMU.get_heading());
	move(41,0.2,0.3,0.2);
	pros::delay(50);
	moveBack(41,0.2,0.3,0.2);
	lastPosition = intake.get_position(); 
	pros::delay(50);
	turn(177,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(55,0.2,0.3,0.2);
	pros::delay(500);
	turn(57,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(25,0.2,0.3,0.2);
	lastPosition = intake.get_position();
	pros::delay(50);
	turn(-20,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBack(30,0.2,0.3,0.2);
	pros::delay(50);
	clampRelease();
	pros::delay(50);
	intake.move_velocity(0);
	pros::delay(50);
	move(13,0.2,0.3,0.2);
	pros::delay(50);
	turn(91.5,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBackSpeclial(110,0.1,0.150,0,90);
	pros::delay(900);
	clampDown();
	intake.move_velocity(500);
	pros::delay(50);
	turn(1,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(40,0.2,0.3,0.3);
	pros::delay(50);
	turn(-90,2.5,0.2,0.1,1,1);
	pros::lcd::print(0,"IMU degrees %f",IMU.get_heading());
	pros::delay(50);
	move(39,0.2,0.3,0.2);
	pros::delay(50);
	turn(-27.5,2.5,0.2,0.1,1,1);
	pros::delay(50);
	pros::lcd::print(0,"IMU degrees %f",IMU.get_heading());
	move(42,0.2,0.3,0.2);
	pros::delay(50);
	moveBack(30,0.2,0.3,0.2);
	pros::delay(50);
	turn(-177,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(55,0.2,0.3,0.2);
	pros::delay(500);
	turn(-50,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(15,0.2,0.3,0.2);
	pros::delay(50);
	turn(45,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBack(10,0.2,0.3,0.2);
	pros::delay(50);
	clampRelease();
	pros::delay(50);
	move(30,0.2,0.3,0.2);
	pros::delay(50);
	turn(0,2.5,0.2,0.1,1,1);
	pros::delay(50);
	intake.move_velocity(0);
	move(130,0.1,0.7,0.5);
	pros::delay(50);
	turn(-125,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBack(60,0.2,0.3,0.2);
	pros::delay(500);
	clampDown();
	pros::delay(50);
	turn(135,2.5,0.2,0.1,1,1);
	pros::delay(50);
	clampRelease();
	pros::delay(50);
	moveBack(80,0.2,0.3,0.2);
	pros::delay(50);
	turn(135,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(10,0.2,0.3,0.2);
	pros::delay(50);
	turn(-87.5,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBack(200,0.2,0.3,0.2);
	clampDown();
	turn(180,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBack(300,0.1,0.7,0.2);

}
void skills2(){
	IMU.set_rotation(0);
	pros::lcd::print(0,"IMU degrees at the start %f",IMU.get_rotation());
    intake.move_velocity(500);
    pros::delay(500);
	move(5,0.2,0.3,0.2);
	pros::delay(50);
    turn(-110,2.5,0.15,0.1,1,1);
	pros::lcd::print(1,"rotated after first turn %f", IMU.get_rotation());
    pros::delay(50);
    moveBack(43,0.2,0.3,0.2);
	pros::delay(450);
	clampDown();
	pros::delay(50);
	turn(0,2.5,0.15,0.1,1,1);
	pros::lcd::print(2,"IMU degrees after %f",IMU.get_rotation());
	pros::delay(50);
	move(45,0.2,0.3,0.2);
	double lastPosition = intake.get_position(); 
	pros::delay(50);
	turn(88,2.5,0.1,0.1,1,1);
	pros::lcd::print(0,"IMU degrees %f",IMU.get_heading());
	pros::delay(50);
	move(41,0.2,0.3,0.2);
	lastPosition = intake.get_position(); 
	pros::delay(50);
	turn(28.5,2.5,0.2,0.1,1,1);
	pros::delay(50);
	pros::lcd::print(0,"IMU degrees %f",IMU.get_heading());
	move(37,0.2,0.3,0.2);
	pros::delay(200);
	turn(-15,2.5,0.2,0.1,1,1);
	while(Rsensor.get_position() < 9000){
            Arm.move_velocity(-50);
            Arm2.move_velocity(-50);
            driveFunc(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
            pros::lcd::print(1,"%f", Rsensor.get_angle());
         }
         Arm.move_velocity(0);
         Arm2.move_velocity(0);
         Arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
         Arm2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	move(43,0.2,0.3,0.2);
	pros::delay(50);
	turn(134,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(43,0.2,0.3,0.2);
	intake.move_velocity(0);
	while(Rsensor.get_position() < 71000){
         Arm.move_velocity(-500);
         Arm2.move_velocity(-500);
         driveFunc(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
      }
      Arm.move_velocity(0);
      Arm2.move_velocity(0);
      Arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      Arm2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	  pros::delay(50);
	moveBack(15,0.2,0.3,0.2);
	pros::delay(50);
	while(Rsensor.get_position() > 9000){
            Arm.move_velocity(500);
            Arm2.move_velocity(500);
            driveFunc(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
         }
         Arm.move_velocity(0);
         Arm2.move_velocity(0);
         Arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
         Arm2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lastPosition = intake.get_position(); 
	intake.move_velocity(500);
	pros::delay(50);
	turn(175,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(140,0.2,0.7,0.2);
	pros::delay(50);
	turn(57,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(25,0.2,0.3,0.2);
	lastPosition = intake.get_position();
	pros::delay(50);
	turn(-30,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBack(10,0.2,0.3,0.2);
	pros::delay(50);
	clampRelease();
	pros::delay(50);
	intake.move_velocity(0);
	pros::delay(50);
	turn(92.5,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBackSpeclial(110,0.1,0.150,0,90);
	pros::delay(900);
	clampDown();
	intake.move_velocity(500);
	pros::delay(50);
	turn(1,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(40,0.2,0.3,0.3);
	pros::delay(50);
	turn(-90,2.5,0.2,0.1,1,1);
	pros::lcd::print(0,"IMU degrees %f",IMU.get_heading());
	pros::delay(50);
	move(35,0.2,0.3,0.2);
	pros::delay(50);
	turn(-27.5,2.5,0.2,0.1,1,1);
	pros::delay(50);
	pros::lcd::print(0,"IMU degrees %f",IMU.get_heading());
	move(42,0.2,0.3,0.2);
	pros::delay(50);
	moveBack(30,0.2,0.3,0.2);
	pros::delay(50);
	turn(-175,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(55,0.2,0.3,0.2);
	pros::delay(500);
	turn(-50,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(15,0.2,0.3,0.2);
	pros::delay(50);
	turn(45,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBack(10,0.2,0.3,0.2);
	pros::delay(50);
	clampRelease();
	pros::delay(50);
	move(30,0.2,0.3,0.2);
	pros::delay(50);
	turn(0,2.5,0.2,0.1,1,1);
	pros::delay(50);
	intake.move_velocity(0);
	move(130,0.1,0.7,0.5);
	pros::delay(50);
	turn(-125,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBack(60,0.2,0.3,0.2);
	pros::delay(500);
	clampDown();
	pros::delay(50);
	turn(135,2.5,0.2,0.1,1,1);
	pros::delay(50);
	clampRelease();
	pros::delay(50);
	moveBack(80,0.2,0.3,0.2);
	pros::delay(50);
	turn(135,2.5,0.2,0.1,1,1);
	pros::delay(50);
	move(20,0.2,0.3,0.2);
	pros::delay(50);
	turn(-70,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBack(200,0.2,0.3,0.2);
	clampDown();
	turn(180,2.5,0.2,0.1,1,1);
	pros::delay(50);
	moveBack(300,0.1,0.7,0.2);

}
void specialOdomSkills(){
	XPos = 0;
	YPos = 0;
	//IMU.set_rotation(-90);
	movePosition(24,12,true);
	pros::delay(50);
	movePosition(24,36,false);
	pros::delay(50);
	movePosition(48,36,false);
}