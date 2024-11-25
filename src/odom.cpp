#include "main.h"
#include "MotorInit.h"
#include "odom.h"
#include "PIDControls.h"

volatile double XPos;
volatile double YPos;
double previous;
double moved = ((RM_MOTOR.get_position() + LM_MOTOR.get_position())/2) * driveTicksPerInch;
void recordPosition(){//repeatdly call
IMU.set_rotation(0);
while(true){
    moved = ((RM_MOTOR.get_position() + LM_MOTOR.get_position())/2) * driveTicksPerInch-moved;
    double currenttheta = IMU.get_rotation();
    double temp = cos(currenttheta) * moved;
     if (temp == std::nan("")) {
        //pros::lcd::print(0,"threading%f, %f",cos(currenttheta), moved);
        pros::delay(5000);
        continue;
    }
       XPos +=temp;
       YPos += (sin(currenttheta) * moved);
       previous = moved;
        pros::lcd::print(0,"threading1%f, %f",temp, XPos);
        pros::delay(5000);
}
}
void movePosition(double targetX, double targetY, bool faceBack){
    double targetTheta = atan((targetY-YPos)/(targetX-XPos));
    if(faceBack == true){
        turn(targetTheta-180, 0.25,0.2,0.1,1,1);
        moveBack(sqrt(((targetX-XPos)*(targetX-XPos)) - ((targetY-YPos)*(targetY-YPos))),0.2,0.3,0.2);
    }
    else{
        turn(targetTheta, 0.25,0.2,0.1,1,1);
        move(sqrt(((targetX-XPos)*(targetX-XPos)) - ((targetY-YPos)*(targetY-YPos))),0.2,0.3,0.2);
    }
}
void testThread(){
    pros::lcd::print(0,"Thread Called");
    pros::delay(50);
}