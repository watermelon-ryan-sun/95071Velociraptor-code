#ifndef _PROS_MORTORINIT_H_
#define _PROS_MORTORINIT_H_


#include "main.h"


static pros::Controller master(pros::E_CONTROLLER_MASTER);
static pros::Motor RB_MOTOR(11,pros::E_MOTOR_GEAR_BLUE, false); //right side, reversed for correct drive
static pros::Motor RF_MOTOR(19,pros::E_MOTOR_GEAR_BLUE, false);
static pros::Motor RM_MOTOR(18,pros::E_MOTOR_GEAR_BLUE, false);


static pros::Motor LB_MOTOR(10,pros::E_MOTOR_GEAR_BLUE, true);
static pros::Motor LF_MOTOR(3,pros::E_MOTOR_GEAR_BLUE, true);
static pros::Motor LM_MOTOR(7,pros::E_MOTOR_GEAR_BLUE, true);

static pros::IMU IMU(1);
static pros::Optical Optical(20);
static pros::ADIDigitalOut clamper ('A'); // extending pulls clamp up
static pros::Rotation Rsensor(15,true);
static pros::Motor intake(8, pros::E_MOTOR_GEAR_GREEN, false);
static pros::Motor Arm(17, pros::E_MOTOR_GEAR_RED, false);
static pros::ADIDigitalOut flipOut('B');//flipout for arm, 
static pros::Rotation Odometry(5);
static pros::ADIDigitalOut sweep('C');
static pros::Distance DSensor(6);
static pros::Motor Arm2(16,pros::E_MOTOR_GEAR_RED, true);
#endif // _PROS_MORTORINIT_H_