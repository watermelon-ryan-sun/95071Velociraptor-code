#include "main.h"
#include "MotorInit.h"
#include "odom.h"
/*double odomCalibration(){
    double currentTheta = IMU.get_heading();
    double deltaTheta = IMU.get_rotation() - lastTheta;
    double drifted = Odom.get_rotation /100;//turns the amount of degrees spun from degrees into radians
    drifted *= 3.1415;/* give us the amount traveled from the drifting*/
   /*drifted /= 180;
    drifted *= radiusOfWheel;
    if(drifted >= 0){
        
    }
}*/
/*void recordPosition(){//use after every move when you turn
    double currenttheta = IMU.get_heading();
    double GlobalX = 0;
    double GlobalY = 0;
    GlobalX += cos(currenttheta) * Odom.get_rotation();
    GlobalY += sin(currenttheta) * Odom.get_rotation();
}*/