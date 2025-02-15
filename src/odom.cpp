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
volatile double avgThetaForArc = 0;
volatile double deltaTheta = 0;
volatile double deltaYLocal = 0;
volatile double deltaXGlobal = 0;
double RM_position = 0, LM_position = 0, BM_position = 0;
double RMPrevPos = 0, LMPrevPos = 0, BMPrevPos = 0;

void recordPosition(){//repeatdly call
    Odometry.set_position(0);
    Odometry.set_data_rate(5);
    tareMotors();
    pros::delay(3000);
    double RM_position = 0, LM_position = 0;
    double RM_moved = 0, LM_moved = 0, BM_moved = 0;
    double halfDeltaTheta = 0;
    //double avgThetaForArc = 0;
    RM_MOTOR.tare_position();
    LM_MOTOR.tare_position();
    //The changes in the X and Y positions (INCHES)
    double deltaXLocal = deltaXLocal;

    //The X and Y offsets converted from their local forms (INCHES)
    double deltaYGlobal = 0;
    int nanCount = 0;
    while(true){
        /*
        if (abs(deltaXGlobal == std::nan("")) || abs(deltaYGlobal == std::nan(""))) {
             continue;
         }
        */

        // TODO: Here, we do not track Left and Right travelling and do not calculate the drift.
        // It uses (L + R)/2 to simulate the tank tracking center's move.        
        RM_position = RM_MOTOR.get_position();
        LM_position = LM_MOTOR.get_position();
        BM_position = Odometry.get_position();

        // Convert to inch
        RM_moved = (RM_position - RMPrevPos) / driveTicksPerInch;
        LM_moved = (LM_position - LMPrevPos) / driveTicksPerInch;
        BM_moved = (BM_position - BMPrevPos) / 5683;
        //make an average of LM moved and Rm moved
        RMPrevPos = RM_position;
        LMPrevPos = LM_position;
        BMPrevPos = BM_position;
    
        double currentAngle = IMU.get_heading() * M_PI / 180.0;
        if (currentAngle == std::nan("")) {
            nanCount ++;
            if (nanCount % 1000 == 0) {
                pros::lcd::print(3,"XPos %f", XPos);
            }
            continue;
        }
        //currentAngle = (LM_moved - RM_moved)/(RTrackRadius+LTrackRadius);
        if (currentAngle < 0) {
            currentAngle += 2 * M_PI;
        } else if (currentAngle > 2 * M_PI) {
            currentAngle -= 2 * M_PI;
        }

        deltaTheta = currentAngle - prevAngle;
        halfDeltaTheta = deltaTheta / 2.0;
        prevAngle = currentAngle;

        // If the deltaTheta is too much, If we didn't turn, then we only translated
        if(abs(deltaTheta) <= M_PI/180) {
            deltaXLocal = BM_moved;
            // could be either L or R, since if deltaTheta == 0 we assume they're =
            deltaYLocal = LM_moved;
            halfDeltaTheta = 0;
        } else {  //Else, caluclate the new local position
            //Calculate the changes in the X and Y values (INCHES)
            //General equation is:
                //Distance = 2 * Radius * sin(deltaTheta / 2)
            deltaXLocal = 2 * sin(halfDeltaTheta) * ((BM_moved / deltaTheta));
            deltaYLocal = 2 * sin(halfDeltaTheta) * ((RM_moved / deltaTheta));
        }


        //The average angle of the robot during it's arc (RADIANS)
        avgThetaForArc = currentAngle - halfDeltaTheta;

        deltaYGlobal = (deltaYLocal * cos(avgThetaForArc)) - (deltaXLocal * sin(avgThetaForArc));
        //deltaYGlobal = deltaYLocal;
        deltaXGlobal = (deltaYLocal * sin(avgThetaForArc)) + (deltaXLocal * cos(avgThetaForArc));
        if (abs(deltaXGlobal) == std::nan("") || abs(deltaYGlobal) == std::nan("")) {
             continue;
        }
        if (deltaXGlobal > 30.0) {
            pros::lcd::print(7,"jumped");
            pros::lcd::print(1,"thetaArcJumped:%f",avgThetaForArc);
            pros::lcd::print(2,"X,%f, Y,%f", LM_MOTOR.get_position(),LMPrevPos);
            std::cout << "x" << deltaYLocal << std::endl;
        }

        XPos += deltaXGlobal;
        YPos += deltaYGlobal;

        //pros::lcd::print(3,"%f, %f, %f", deltaTheta,RM_moved, LM_moved);
        //pros::lcd::print(4,"%f",avgThetaForArc);
        //master.print(2,3,"important %f", YPos);
    }
}
/*void JAR_position(float ForwardTracker_position, float SidewaysTracker_position, float orientation_deg){
  // this-> always refers to the old version of the variable, so subtracting this->x from x gives delta x.
  float Forward_delta = ForwardTracker_position-this->ForwardTracker_position;
  float Sideways_delta = SidewaysTracker_position-this->SideWaysTracker_position;
  this->ForwardTracker_position=ForwardTracker_position;
  this->SideWaysTracker_position=SidewaysTracker_position;
  float orientation_rad = to_rad(orientation_deg);
  float prev_orientation_rad = to_rad(this->orientation_deg);
  float orientation_delta_rad = orientation_rad-prev_orientation_rad;
  this->orientation_deg=orientation_deg;

  float local_X_position;
  float local_Y_position;

  if (orientation_delta_rad == 0) {
    local_X_position = Sideways_delta;
    local_Y_position = Forward_delta;
  } else {
    local_X_position = (2*sin(orientation_delta_rad/2))*((Sideways_delta/orientation_delta_rad)+SidewaysTracker_center_distance); 
    local_Y_position = (2*sin(orientation_delta_rad/2))*((Forward_delta/orientation_delta_rad)+ForwardTracker_center_distance);
  }

  float local_polar_angle;
  float local_polar_length;

  if (local_X_position == 0 && local_Y_position == 0){
    local_polar_angle = 0;
    local_polar_length = 0;
  } else {
    local_polar_angle = atan2(local_Y_position, local_X_position); 
    local_polar_length = sqrt(pow(local_X_position, 2) + pow(local_Y_position, 2)); 
  }

  float global_polar_angle = local_polar_angle - prev_orientation_rad - (orientation_delta_rad/2);

  float X_position_delta = local_polar_length*cos(global_polar_angle); 
  float Y_position_delta = local_polar_length*sin(global_polar_angle);

  X_position+=X_position_delta;
  Y_position+=Y_position_delta;
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
