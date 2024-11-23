#include "main.h"
#include "MotorInit.h"
#include "odom.h"
#include "PIDControls.h"

double XPos;
double YPos;
void recordPosition(){//repeatdly call
while(true){
    double currenttheta = IMU.get_rotation();
        XPos +=(cos(currenttheta) * (Odometry.get_position()*(3.1415/180)));
        YPos += (sin(currenttheta) * (Odometry.get_position()*(3.1415/180)));
        pros::lcd::print(0,"threading%f", XPos);
        pros::delay(500);
}
}
void movePosition(double targetX, double targetY, bool faceBack){
    double targetTheta = tan((targetY-YPos)/(targetX-XPos));
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