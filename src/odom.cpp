#include "main.h"
#include "MotorInit.h"
#include "odom.h"
#include "PIDControls.h"

//Distances of tracking wheels from tracking center (INCHES)
static const double LTrackRadius = 5;//~5 now
static const double RTrackRadius = 5;
static const double BTrackRadius = 3.5;

volatile double XPos = 0;
volatile double YPos = 0;
volatile double currentAngle = M_PI / 2.0;  // current heading in radians
volatile double prevAngle = 0; // previous heading in radians
double RM_position = 0, LM_position = 0, BM_position = 0;
double RMPrevPos = 0, LMPrevPos = 0, BMPrevPos = 0;

void recordPosition(){//repeatdly call
    tareMotors();

    double RM_position = 0, LM_position = 0;
    double RM_moved = 0, LM_moved = 0, BM_moved = 0;
    double deltaTheta = 0, halfDeltaTheta = 0;
    double avgThetaForArc = 0;
    RM_MOTOR.tare_position();
    LM_MOTOR.tare_position();
    //The changes in the X and Y positions (INCHES)
    double deltaXLocal = 0;
    double deltaYLocal = 0;

    //The X and Y offsets converted from their local forms (INCHES)
    double deltaXGlobal = 0;
    double deltaYGlobal = 0;
    pros::delay(2000);
    while(true){
        pros::delay(10);
        if ((deltaXGlobal == std::nan("")) || (deltaYGlobal == std::nan(""))) {
             //pros::lcd::print(0,"threading%f, %f",cos(currenttheta), moved);
             pros::delay(50);
             continue;
         }
        // TODO: Here, we do not track Left and Right travelling and do not calculate the drift.
        // It uses (L + R)/2 to simulate the tank tracking center's move.        
        RM_position = RM_MOTOR.get_position();
        LM_position = LM_MOTOR.get_position();
        //BM_position = BM_MOTOR.get_position();

        // Convert to inch
        RM_moved = (RM_position - RMPrevPos) / driveTicksPerInch;
        LM_moved = (LM_position - LMPrevPos) / driveTicksPerInch;
        BM_moved = (BM_position - BMPrevPos) / driveTicksPerInch;

        RMPrevPos = RM_position;
        LMPrevPos = LM_position;
        BMPrevPos = BM_position;

        //pros::lcd::print(0,"moved%f, %f, %f", moved, RM_position, LM_position);
        double currentAngle = (90 - IMU.get_rotation()) * M_PI / 180.0;
        currentAngle = (LM_moved - RM_moved)/(RTrackRadius+LTrackRadius);
        if (currentAngle < 0) {
            currentAngle += 2 * M_PI;
        } else if (currentAngle > 2 * M_PI) {
            currentAngle -= 2 * M_PI;
        }

        deltaTheta = currentAngle - prevAngle;
        halfDeltaTheta = deltaTheta / 2.0;
        prevAngle = currentAngle;

        // If the deltaTheta is too much, If we didn't turn, then we only translated
        if(abs(deltaTheta) <= M_PI/360) {
            deltaXLocal = BM_moved;
            // could be either L or R, since if deltaTheta == 0 we assume they're =
            deltaYLocal = LM_moved;
            halfDeltaTheta = 0;
        } else {  //Else, caluclate the new local position
            //Calculate the changes in the X and Y values (INCHES)
            //General equation is:
                //Distance = 2 * Radius * sin(deltaTheta / 2)
            deltaXLocal = 2 * sin(halfDeltaTheta) * ((BM_moved / deltaTheta) + BTrackRadius);
            deltaYLocal = 2 * sin(halfDeltaTheta) * ((RM_moved / deltaTheta) + RTrackRadius);
        }


        //The average angle of the robot during it's arc (RADIANS)
        avgThetaForArc = currentAngle - halfDeltaTheta;

        deltaYGlobal = (deltaYLocal * cos(avgThetaForArc)) - (deltaXLocal * sin(avgThetaForArc));
        deltaXGlobal = (deltaYLocal * sin(avgThetaForArc)) + (deltaXLocal * cos(avgThetaForArc));
        XPos += deltaXGlobal;
        YPos += deltaYGlobal;

        pros::lcd::print(3,"%f, %f, %f", deltaTheta,RM_moved, LM_moved);
        pros::lcd::print(4,"%f",avgThetaForArc);
        pros::delay(60);
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
    double targetTheta;
    double targetDistance;

    if(faceBack == true){
        //while(abs(targetX - XPos) > 0.1 && abs(targetY - YPos) > 0.1){
            targetTheta = 180 * atan((targetY-YPos)/(targetX-XPos))/M_PI;
            targetDistance = sqrt(((targetX-XPos)*(targetX-XPos)) + ((targetY-YPos)*(targetY-YPos)));
            pros::lcd::print(0,"target Theta/dist %f, %f", targetTheta, targetDistance);
            turn(targetTheta-180, 2.5,0.2,0.1,1,1);
            pros::delay(50);
            moveBack(targetDistance,0.1,0.1,0.1);
            pros::delay(50);
        //}
    }
    else{
        //pros::lcd::print(1,"target Theta/dist %f, %f", targetX, YPos);
        //pros::lcd::print(0,"target Theta/dist1 %f, %f", targetY, XPos);
        //while(abs(targetY - YPos) > 12 || abs(targetX - XPos) > 12){
        if(targetX == XPos){
            targetTheta = 90;
        }
        else{
            if(targetX - XPos > 0 && targetY - YPos > 0){
                targetTheta = 180 * atan((targetY-YPos)/(targetX-XPos))/M_PI;
            }
            else if(targetX - XPos < 0 && targetY - YPos > 0){
                targetTheta = ((90 - (180 * atan((targetY-YPos)/(targetX-XPos)))/M_PI));
            }
            else if(targetX - XPos < 0 && targetY - YPos < 0){
                targetTheta = ( 180 - 180 * atan((targetY-YPos)/(targetX-XPos))/M_PI);
            }
            else if(targetX - XPos > 0 && targetY - YPos < 0){
                targetTheta = ( 270 - 180* atan((targetY-YPos)/(targetX-XPos))/M_PI);
            }
            }
        turn(targetTheta, 2.5,0.2,0.1,1,1);
        pros::delay(10);
        targetDistance = sqrt(((targetX-XPos)*(targetX-XPos)) + ((targetY-YPos)*(targetY-YPos)));
        move(targetDistance,0.2, 0.1,0.2);
        pros::delay(100);
        master.print(2,3,"%f, %f ,%f", targetDistance, YPos, IMU.get_rotation());
        pros::lcd::print(5,"%f,%f,%f", XPos, YPos, targetTheta);
        //master.print(3,3," %f", IMU.get_heading());
        //}

    }
}
void testThread(){
    pros::lcd::print(0,"Thread Called");
    pros::delay(50);
}