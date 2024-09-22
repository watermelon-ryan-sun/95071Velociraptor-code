#ifndef _PROS_PIDCONTROLS_H_
#define _PROS_PIDCONTROLS_H_


static const double driveTicksPerInch = 9.42477796077;
static const double kD = 0.2;
static const double kP = 0.1;
static const double kI = 1;
static const double rotationInit = 0.0;
static const double radiusCenter = 11.258330; //For calculating the arc length
static const double rpmToTps = 5;


void tareMotors();
void turn(double heading,double miliseconds);
void move(double distance);
void move_straight(double distance);
void RunIntake(double target);
void stopMotors();
void moveRight(double output);
void moveLeft(double output);
#endif // _PROS_PIDCONTROLS_H_