#ifndef _PROS_FUNCTIONS_H_
#define _PROS_FUNCTIONS_H_


//global variables
static int amountOfAuton = 0;
static bool mode2 = true;
static bool mode3 = true;
static double prevPower = 0;
void autonSelector();
void driveFunc(double power, double turn);
void clampDown();
void clampRelease();
void driveIntake();
void moveIntake();
void moveArm();
void clampTeleOP();
void sigmaFlipOut185();
void slowIntake();
void autonSelector();
#endif // _PROS_FUNCTIONS_H_
