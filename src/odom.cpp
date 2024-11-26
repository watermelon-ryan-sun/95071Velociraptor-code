#include "main.h"
#include "MotorInit.h"
#include "odom.h"
#include "PIDControls.h"

volatile double XPos;
volatile double YPos;
double previous;
double moved = ((RM_MOTOR.get_position() + LM_MOTOR.get_position())/2) / driveTicksPerInch;
void recordPosition(){//repeatdly call
    IMU.set_rotation(0);
    while(true){
        // TODO: Here, we do not track Left and Right travelling and do not calculate the drift.
        // It uses (L + R)/2 to simulate the tank tracking center's move.
        moved = (((RM_MOTOR.get_position() + LM_MOTOR.get_position())/2) / driveTicksPerInch) - previous;


        double currenttheta = IMU.get_rotation();
        double xMoved = sin(currenttheta) * moved;
        double yMoved = cos(currenttheta) * moved;
        // Skip the one with error
        if ((xMoved == std::nan("")) || (yMoved == std::nan(""))) {
            //pros::lcd::print(0,"threading%f, %f",cos(currenttheta), moved);
            pros::delay(50);
            continue;
        }
        XPos += xMoved;
        YPos += yMoved;
        previous = moved;
        pros::lcd::print(0,"threading1%f, %f", XPos, YPos);
        pros::delay(50);
    }
}

// Use Tao's algorithm
/* TO BE IMPLEMENTED
void recordPosition2(){
    IMU.set_rotation(0);
    double angle = 0, halfAngle = 0,  L = 0, R = 0;
    double lastYL = 0, lastYR = 0;
    double currentYL = 0, currentYR = 0;
    double yMoved = 0;
    while(true){
        currentYL = LM_MOTOR.get_position() / driveTicksPerInch;
        currentYR = RM_MOTOR.get_position()/ driveTicksPerInch;
        angle = (L - R) / (verticalOffset1 + verticalOffset2);
        if (angle != 0) {
            halfAngle = angle / 2.0;
            yMoved = 2.0 * sin(halfAngle) * ((R / angle) + verticalOffset2);

        } else {
            halfAngle = 0.0;
            yMoved = R;
        }
        moved = (((RM_MOTOR.get_position() + LM_MOTOR.get_position())/2) / driveTicksPerInch) - previous;


        double currenttheta = IMU.get_rotation();
        double xMoved = cos(currenttheta) * moved;
        double yMoved = sin(currenttheta) * moved;
        // Skip the one with error
        if ((xMoved == std::nan("")) || (yMoved == std::nan(""))) {
            //pros::lcd::print(0,"threading%f, %f",cos(currenttheta), moved);
            pros::delay(50);
            continue;
        }
        XPos +=(cos(currenttheta)*moved);
        YPos += (sin(currenttheta) * moved);
        previous = moved;
        pros::lcd::print(0,"threading1%f, %f", XPos, YPos);
        pros::delay(50);
    }
}
*/

void movePosition(double targetX, double targetY, bool faceBack){
    double targetTheta = 0.0;
    double targetDistance = 0.0;

    if(faceBack == true){
        while(abs(targetX - XPos) < 0.1 && abs(targetY - YPos) < 0.1){
            targetTheta = 180 * atan((targetY-YPos)/(targetX-XPos))/M_PI;
            targetDistance = sqrt(((targetX-XPos)*(targetX-XPos)) + ((targetY-YPos)*(targetY-YPos)));
            pros::lcd::print(0,"target Theta/dist %f, %f", targetTheta, targetDistance);
            turn(targetTheta-180, 2.5,0.2,0.1,1,1);
            pros::delay(100);
            moveBack(targetDistance,0.2,0.3,0.2);
            pros::delay(100);
        }
    }
    else{
        while(abs(targetX - XPos) < 0.1 && abs(targetY - YPos) < 0.1){
            targetTheta = 180 * atan((targetY-YPos)/(targetX-XPos))/M_PI;
            targetDistance = sqrt(((targetX-XPos)*(targetX-XPos)) + ((targetY-YPos)*(targetY-YPos)));
            pros::lcd::print(0,"target Theta/dist %f, %f", targetTheta, targetDistance);

            turn(targetTheta, 2.5, 0.2, 0.1, 1,1);
            pros::delay(100);
            move(targetDistance, 0.2, 0.3, 0.2);
            pros::delay(100);
        }
    }
}
void testThread(){
    pros::lcd::print(0,"Thread Called");
    pros::delay(50);
}