#include "main.h"
#include "MotorInit.h"
#include "odom.h"
#include "PIDControls.h"

volatile double XPos = 0;
volatile double YPos = 0;
double moved = 0;
double RM_moved = 0, LM_moved = 0;
volatile double currentAngle = 0;  // current heading in radians
volatile double prevAngle = 0; // previous heading in radians
double RM_position = 0, LM_position = 0, BM_Position = 0;
double RMPrevPos = 0, LMPrevPos = 0, BMPrevPos = 0;
void recordPosition(){//repeatdly call
    double RM_position = 0, LM_position = 0;
    IMU.set_rotation(0);
    RM_MOTOR.tare_position();
    LM_MOTOR.tare_position();
    double targetHeading = 0;//plan to make volatile, so that different moves can alter it to their target
    double prevRight = 0;
    double prevLeft = 0;
    double leftFromRight = 0;
    while(true){
        // TODO: Here, we do not track Left and Right travelling and do not calculate the drift.
        // It uses (L + R)/2 to simulate the tank tracking center's move.
        pros::delay(5000);        
        RM_position = RM_MOTOR.get_position();
        LM_position = LM_MOTOR.get_position();
        RM_moved = RM_position - prevRight;
        LM_moved = LM_position - prevLeft;
        //pros::lcd::print(0,"moved%f, %f, %f", moved, RM_position, LM_position);
        double currenttheta = IMU.get_rotation();
        currenttheta -= 90;
        if(targetHeading == currenttheta){
        double xMoved = cos(currenttheta) * RM_moved;
        double yMoved = sin(currenttheta) * LM_moved;
        // Skip the one with error
            if ((xMoved == std::nan("")) || (yMoved == std::nan(""))) {
                //pros::lcd::print(0,"threading%f, %f",cos(currenttheta), moved);
                pros::delay(50);
                continue;
                }
            YPos += xMoved;
            XPos += yMoved;
        }
        else{//arc'd to the right
            double arcTheta = (RM_moved - prevRight)/((RM_moved-prevRight)-(LM_moved-prevLeft))/((RM_moved-prevRight)+leftFromRight);
            double radiusRight = 0;// arc/arctheta
            double radiusLeft = 0;// arcLeft/arctheta
            double length = (sin(arcTheta/2)*radiusCenter)*2;
            XPos += (cos(currenttheta)*length);
            YPos += (sin(currenttheta)*length);
        }
        //pros::lcd::print(0,"threading1%f, %f", XPos, YPos);
        pros::delay(50);
        //master.print(2,3,"important %f", YPos);
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
        //while(abs(targetX - XPos) > 0.1 && abs(targetY - YPos) > 0.1){
            targetTheta = 180 * atan((targetX-XPos)/(targetY-YPos))/M_PI;
            targetDistance = sqrt(((targetX-XPos)*(targetX-XPos)) + ((targetY-YPos)*(targetY-YPos)));
            pros::lcd::print(0,"target Theta/dist %f, %f", targetTheta, targetDistance);
            turn(targetTheta-180, 2.5,0.2,0.1,1,1);
            pros::delay(100);
            moveBack(targetDistance,0.2,0.3,0.2);
            pros::delay(100);
        //}
    }
    else{
        //pros::lcd::print(1,"target Theta/dist %f, %f", targetX, YPos);
        //pros::lcd::print(0,"target Theta/dist1 %f, %f", targetY, XPos);
        //while(abs(targetY - YPos) > 12 || abs(targetX - XPos) > 12){
            targetTheta = 180 * atan((targetY-YPos)/(targetX-XPos))/M_PI;
            targetDistance = (sqrt(((targetX-XPos)*(targetX-XPos)) + ((targetY-YPos)*(targetY-YPos))));
            pros::delay(100);
            move(targetDistance, 0.1, 0.2, 0.2);
            pros::delay(100);
            master.print(2,3,"%f, %f ,%f", targetDistance, YPos, IMU.get_rotation());
            //master.print(3,3," %f", IMU.get_heading());
        //}

    }
}
void testThread(){
    pros::lcd::print(0,"Thread Called");
    pros::delay(50);
}