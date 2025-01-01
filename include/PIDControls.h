#ifndef _PROS_PIDCONTROLS_H_
#define _PROS_PIDCONTROLS_H_


static const double driveTicksPerInch = 17.9054591756; //long distance constants, short range  0.1, 0.2, 0.1 (9.42477796077*50)/24
static const double inchToTicks = 3.141592/150;
static const double rotationInit = 0.0;
static const double radiusCenter = 10; //For calculating the arc length
static const double rpmToTps = 6;


void tareMotors();
void turn(double heading, double Kp, double Kd, double Ki,double O, double U);
void move(double distance,double kP, double kI, double kD);
void move_straight(double distance);
void RunIntake(double target);
void stopMotors();
void moveRight(double output);
void moveLeft(double output);
void moveBack(double distance, double kP, double kI, double kD);
void moveBackSpeclial(double distance, double kP, double kI, double kD, double targetHead);
void PIDArm();
void PIDIntake();
void putDownArm();
void ringInArm();
#endif // _PROS_PIDCONTROLS_H_