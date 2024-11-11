#include "main.h"
#include "MotorInit.h"
#include "odom.h"
#include "PIDControls.h"
void recordPosition(){//use after every move when you turn
    double currenttheta = IMU.get_heading();
    XPos += (cos(currenttheta) * (Odometry.get_position()*(3.1415/180)));
    YPos += (sin(currenttheta) * (Odometry.get_position()*(3.1415/180)));
}
void AdjustPosition(double TheoreticalX, double TheoreticalY){
    double neededTurn = 0;
    if(XPos < TheoreticalX && YPos < TheoreticalY){
        neededTurn = (atan(1) - IMU.get_heading());
        turn((neededTurn*180/3.1415),2,1,0.5,1,1);
        //();
    }
}