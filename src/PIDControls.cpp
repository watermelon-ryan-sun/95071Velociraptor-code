#include "PIDControls.h"
#include "MotorInit.h"
#include "Odom.h"
void tareMotors() {
   LF_MOTOR.tare_position();
   LM_MOTOR.tare_position();
   LB_MOTOR.tare_position();
   RB_MOTOR.tare_position();
   RM_MOTOR.tare_position();
   RF_MOTOR.tare_position();
}
void turn(double heading, double Kp, double Kd, double Ki, double O, double U) { //turns a certain amount of degrees
/*New turn code with IMU*/
double prevX = XPos;
double prevY = YPos;

double error = heading-IMU.get_rotation();
error *= O;
error /= U;
    if(fabs(error) > 180){
        // TODO: Why it is always -360? What if heading is -190?
        error -= 360;
    }
    double prevError = 0;
    double integral = 0;
    double threshold = 20;
    while(true){
        error = heading - IMU.get_rotation();
        integral += error;
        if(fabs(error)>threshold){
            integral = 0;
        }

        double power = error * Kp + integral * Ki + (error-prevError) * Kd * driveTicksPerInch;
        moveRight(-1*power);
        moveLeft(power);

        prevError = error;
        pros::delay(10);
        // TODO: HX, should this be moved after error is calculated?
        // My point is that if the angle is less than 1.0, it should not
        // wait for another turn to break. 
        // 1.0 is too large as well.
        if(abs(error) < 0.01){
            break;
        }
        if(fabs(prevError)-fabs(error)<0.05 && fabs(error)<0.4){
            break;
        }
    }
   stopMotors();
   XPos = prevX;
   YPos=prevY;
   pros::lcd::print(1, "degrees after %f", IMU.get_rotation());
}
void PIDArm(){
    Rsensor.reset_position();
    Rsensor.set_reversed(true);
    double target = 7500;
    double traveled = Rsensor.get_angle();
    //Rsensor.set_data_rate(5);
    double error = target;
    //double lastError = 0.0;
    //double integral = error;
    while(target > traveled){
        Arm.move_velocity(600);
        traveled = Rsensor.get_angle();
        pros::lcd::print(0,"sensor : %f",Rsensor.get_angle());
        pros::delay(10);
    }
    Arm.move_velocity(0);
    Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
void PIDIntake(){
    intake.tare_position();

}
void move2(double targetX, double targetY,double targetTheta,double kP, double kI, double kD) {
    tareMotors();
   double rightOutput = 0.0;
   double leftOutput = 0.0;
   double targetDistance = sqrt((targetX-XPos)*(targetX-XPos) + (targetY+YPos)*(targetY-YPos));//find distance needed
   double errorR = targetDistance *= driveTicksPerInch;
   double errorL = targetDistance *= driveTicksPerInch;
   double rightMeasured = (RM_MOTOR.get_position());
   double leftMeasured = (LM_MOTOR.get_position());
   double distanceTL = 0.0;//area under the curve
   double distanceTR = 0.0;//actual position in ticks
   double integralR = 0.0;
   double integralL = 0.0;
   double prevErrorR = 0;
   double prevErrorL = 0;
   // TODO: HX comment, the following two lines do not do anything, the value calculated is not assigned back.
   // They can be removed.
   while(errorR > distanceTR || errorL > distanceTL){

    integralR = errorR - distanceTR;
    integralL = errorL - distanceTL;

    
    distanceTR = rightMeasured;
    distanceTL = leftMeasured;
    if(integralR > 300){
        integralR = 300;
    }
    if(integralL > 300){
        integralL = 300;
    }
    if(targetTheta!= IMU.get_rotation()){
        rightOutput = ((integralR)*kI + (errorR*kP) + (targetTheta - IMU.get_heading())*kD);//missing length left in ticks 
        leftOutput = ((integralL)*kI + (errorL*kP) - ((targetTheta - IMU.get_heading())));//what does kP do?
    }
    else{
        rightOutput = ((integralR)*kI + (errorR*kP));//missing length left in ticks 
        leftOutput = ((integralL)*kI + (errorL*kP));//what does kP do?
    }
    rightOutput = ((integralR)*kI - (errorR*kP) - (errorR-prevErrorR)*kD);//missing length left in ticks 
    leftOutput = ((integralL)*kI + (errorL*kP) - ((errorL-prevErrorL)*kD));//what does kP do?
    pros::lcd::print(0, "before calling moveRight");
    moveRight(rightOutput);
    pros::lcd::print(0, "after calling moveRight");
    moveLeft(leftOutput);
    rightMeasured = ((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
    leftMeasured = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
    //OdomCalibration();
    pros::delay(10);
   }
   stopMotors();//hit the ideal distance so stop yourself
}
void move(double targetX, double targetY, double kP, double kI, double kD) {
    double OPL = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
    double OPR = ((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
   double rightOutput = 0.0;
   double leftOutput = 0.0;
   double XDiff = targetX - XPos;
   double YDiff = targetY-YPos;
   double distance = sqrt((XDiff*XDiff) + (YDiff*YDiff));
   double targetInches = distance;
   distance *= driveTicksPerInch;
   double targetR = distance + OPR;
   double targetL = distance + OPL;
   double rightMeasured = OPR;
   double leftMeasured = OPL;
   double error = 0.0;//error between the two sides
   double distanceT = 0.0;//area under the curve
   double integralR = 0.0;
   double integralL = 0.0;
   double targetHeading = atan(XDiff/YDiff);
   double currentHeading = (IMU.get_heading()* M_PI)/180;
   // TODO: HX comment, the following two lines do not do anything, the value calculated is not assigned back.
   // They can be removed.
   while(targetR > rightMeasured || targetL > leftMeasured){
    integralR = (targetR - OPR)/driveTicksPerInch;
    integralL = (targetL - OPL)/driveTicksPerInch;
    if(integralR > 300){
        integralL  = 300;
    }
    if(integralL > 300){
        integralL = 300;
    }
    currentHeading = (IMU.get_heading()* M_PI)/180;
    moveRight((integralR*kI) + targetInches * kP - (targetHeading - currentHeading) * kD);
    moveLeft((integralL*kI) + targetInches*kP + (targetHeading - currentHeading) * kD);
    leftMeasured = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
    rightMeasured =((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
}
stopMotors();
}
void moveToPosition(double targetX, double targetY, double kP, double kI, double kD){
    double leftVoltage = 0;
    double rightVoltage = 0;
    double targetTheta = 0;
    double realTheta = 0;
    double distX = targetX - XPos;
    double distY = targetY - YPos;
    while(targetX-XPos > 1 && (targetY-YPos > 1)){
        targetTheta = atan((targetY-YPos)/(targetX-XPos));
        realTheta = IMU.get_heading() * M_PI/180;
        if(realTheta > targetTheta){
            //cut the leftVoltage
            leftVoltage = realTheta - targetTheta;
            rightVoltage = targetTheta-realTheta;//figure out a relationship between the two quickly, ideas: find velocity and then diff between velocities to do something
        }
        RM_MOTOR.move_voltage(rightVoltage);
        LM_MOTOR.move_voltage(leftVoltage);
        pros::delay(50);
    }

}
void moveBack(double distance, double kP, double kI, double kD) {
    double OPL = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
    double OPR = ((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
   double rightOutput = 0.0;
   double leftOutput = 0.0;

   double targetInches = distance;
   distance *= driveTicksPerInch;
   double targetR = OPR - distance;
   double targetL = OPL - distance;
   double rightMeasured = OPR;
   double leftMeasured = OPL;
   double error = 0.0;//error between the two sides
   double distanceT = 0.0;//area under the curve
   double integralR = 0.0;
   double integralL = 0.0;

   // TODO: HX comment, the following two lines do not do anything, the value calculated is not assigned back.
   // They can be removed.
   while(targetR < rightMeasured || targetL < leftMeasured){
    integralR = -(OPR - targetR)/driveTicksPerInch;
    integralL = -(OPL - targetR)/driveTicksPerInch;
    if(integralR > 300){
        integralL  = 300;
    }
    if(integralL > 300){
        integralL = 300;
    }
    moveRight((integralR*kI) - targetInches * kP);
    moveLeft((integralL*kI) - targetInches*kP);
    leftMeasured = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
    rightMeasured = ((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
}
stopMotors();
}
/*void move2(double distance,double kP, double kI, double kD) {
  double error = (distance *= driveTicksPerInch);
  double errorR = error + RM_MOTOR.get_position();
  double errorL = error + LM_MOTOR.get_position();
  double distanceTR = RM_MOTOR.get_position();
  double distanceTL = LM_MOTOR.get_position();
  double integral = 0;
  double deviationR = 0;
  double rightOutput = 0;
  double leftOutput = 0;
  double deviationL = 0;
  while((errorR-distanceTR) > 1 || (errorL-distanceTL) > 1){
    integral += error;
    if(integral > 20){
        integral = 0;
    }
    rightOutput = ((integral*kI) + (deviationR*kD) + (error-distanceTR)* kP);
    leftOutput = ((integral*kI) + (deviationL*kD) + (error-distanceTR)* kP);
    deviationR = 0;
  }
}
/*void move_straight(double distance) {
   double initialHeading = inertial.get_rotation();
   LF_MOTOR.tare_position();
   LM_MOTOR.tare_position();
   LB_MOTOR.tare_position();
   RB_MOTOR.tare_position();
   RM_MOTOR.tare_position();
   RF_MOTOR.tare_position();


   /*distance *= driveTicksPerInch;


   // movement constants
   double kP = 30;
   double kI = 0;
   double kD = 0;
   double integral = 0;
   double lastError = distance;


   // turning constants
   double hkP = 90;
   double hkI = 4;
   double hkD = 200;
   double hintegral = 0;
   double hlastError = 0;


   int restedStates = 0;
   int stalledStates = 0;
   double rightMeasured = ((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
   double leftMeasured = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
   while ((restedStates < 5 && stalledStates < 50)) {
       double error = (distance - (rightMeasured + leftMeasured)/2);
       if (fabs(error) < 2.5*driveTicksPerInch) integral += error;
       if (fabs(error) < 0.05*driveTicksPerInch) integral = 0;


       if (fabs(error - lastError) < 0.05*driveTicksPerInch) stalledStates++;
       else stalledStates = 0;


       double herror = initialHeading - inertial.get_rotation();
       if (fabs(herror) < 30) hintegral += herror;
       if (fabs(herror) < 0.5) hintegral = 0;


       double out = kP * error + kI * integral + kD * (error - lastError);
       double hOut = hkP * herror + hkI * hintegral + hkD * (herror - hlastError);
       double maxOut = fmax(12000, fmax(out - hOut, out + hOut));
       LF_MOTOR.move_voltage((out + hOut)*12000/maxOut);
       LM_MOTOR.move_voltage((out + hOut)*12000/maxOut);
       LB_MOTOR.move_voltage((out + hOut)*12000/maxOut);
       RB_MOTOR.move_voltage((out + hOut)*12000/maxOut);
       RM_MOTOR.move_voltage((out + hOut)*12000/maxOut);
       RF_MOTOR.move_voltage((out + hOut)*12000/maxOut);
      


       lastError = error;
       hlastError = herror;


       pros::delay(10);


       if (fabs(distance - (rightMeasured + leftMeasured)/2) < 0.1*driveTicksPerInch) restedStates++;
       else restedStates = 0;
   }
   stopMotors();
    double rightMeasured = ((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
   double leftMeasured = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
   double target = distance*driveTicksPerInch; //convert to radians, multiply by the radius for arc length, divide by two, cause you're moving both sides in opposite directions
   double error = target - (rightMeasured + leftMeasured)/2;
   double integral = 0;
   double lastError = 0;
   double integralCap = 200;
   double integralMin = 10;//prevention of integral windup
   double tolerableError = 0;//tolerable amount of error before you wanna turn the other way
   while(abs(error) > tolerableError){
       rightMeasured = ((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
       leftMeasured = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
       error = target - (rightMeasured - leftMeasured)/2;
       integral += error;
       if(integral > integralCap){//windup
           integral = integralCap;
       }
       if(integral > integralMin){
           integral = 0;
       }
       double out = error * kP + (integral * kI) + (lastError * kP);
       lastError = error;
       pros::lcd::print(0, "integral = %f", integral);
       pros::lcd::print(0, "error = %f", error);
       pros::lcd::print(0, "target = %f", error);
       RB_MOTOR.move_absolute(out,out);
       RM_MOTOR.move_absolute(out,out);
       RF_MOTOR.move_absolute(out,out);
       LF_MOTOR.move_absolute(out,out);
       LM_MOTOR.move_absolute(out,out);
       LB_MOTOR.move_absolute(out,out);
   }
   stopMotors();
}*/


void RunIntake(double target){
    double blue = 0;//arbiturary values for now
    double red = 0;
    double empty = 200;
    intake.move_velocity(300);
   /*if(true == true){//blue side means side == true
    while(IntakeOptical.get_brightness() == empty){//replace red and blue with detected color values, make sure the no ring detected is right
        intake.move_velocity(50);
    }
    if(IntakeOptical.get_hue() == red){//sputs it out, might want to change later
        intake.move_velocity(-500);
    }
    else if(IntakeOptical.get_hue() == blue){
        intake.move_velocity(500);
    }
   }
   else{
    while(IntakeOptical.get_brightness() == empty){//same thing but reverse
        intake.move_velocity(50);
    }
    if(IntakeOptical.get_hue() == blue){
        intake.move_velocity(-500);
    }
    else if(IntakeOptical.get_hue() == red){
        intake.move_velocity(500);
    }
   }*/
}
void ringInArm(){
    intake.move_velocity(0);
}

void stopMotors(){
   LF_MOTOR.move_voltage(0);
   LM_MOTOR.move_voltage(0);
   LB_MOTOR.move_voltage(0);
   RB_MOTOR.move_voltage(0);
   RM_MOTOR.move_voltage(0);
   RF_MOTOR.move_voltage(0);
}
void moveLeft(double output){
    LB_MOTOR.move_velocity(output);
    LM_MOTOR.move_velocity(output);
    LF_MOTOR.move_velocity(output);
}
void moveRight(double output){
    RB_MOTOR.move_velocity(output);
    RM_MOTOR.move_velocity(output);
    RF_MOTOR.move_velocity(output);
}
void putDownArm(){
    Arm.move_velocity(-300);
    pros::delay(1000);
    Arm.move_velocity(0);
}
void intakeMacro(){//for driving
    while(true){
        intake.move_velocity(0);
        if(Optical.get_proximity() > 0){
            intake.move_velocity(500);
        }
        else{
            break;
        }
    }
    intake.move_velocity(-500);
}