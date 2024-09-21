#include "PIDControls.h"
#include "MotorInit.h"
void tareMotors() {
   LF_MOTOR.tare_position();
   LM_MOTOR.tare_position();
   LB_MOTOR.tare_position();
   RB_MOTOR.tare_position();
   RM_MOTOR.tare_position();
   RF_MOTOR.tare_position();
}


void turn(double heading, double miliseconds) { //turns a certain amount of degrees
   /*double angle = fmod(heading - inertial.get_heading(), 360); //Amigo Code
   if (angle > 180) angle -= 360;
   if (angle < -180) angle += 360;
   double targetAngle = inertial.get_rotation() + angle;


   double lastError = targetAngle - inertial.get_rotation();


   int restedStates = 0;
   int stalledStates = 0;
   double integral = 10;
   while (restedStates < 5 && stalledStates < 50) {
       double error = targetAngle - inertial.get_rotation();
       if (fabs(error) < 30) integral += error;
       if (fabs(error) < 0.1) integral = 0;


       if (fabs(error - lastError) < 0.005) stalledStates++;
       else stalledStates = 0;


       double out = kP * error + kI * integral + kD * (error - lastError);
       RB_MOTOR.move_voltage(-out);
       RM_MOTOR.move_voltage(-out);
       RF_MOTOR.move_voltage(-out);
       LF_MOTOR.move_voltage(out);
       LM_MOTOR.move_voltage(out);
       LB_MOTOR.move_voltage(out);


       lastError = error;


       pros::delay(10);


       if (fabs(targetAngle - inertial.get_rotation()) < 2) restedStates++;
       else restedStates = 0;
   }
   LF_MOTOR.move_voltage(0);
   LM_MOTOR.move_voltage(0);
   LB_MOTOR.move_voltage(0);
   RB_MOTOR.move_voltage(0);
   RM_MOTOR.move_voltage(0);
   RF_MOTOR.move_voltage(0);*/
   tareMotors();
   pros::lcd::print(0, "motor was moved");
   double rightMeasured = 0;
   double leftMeasured = 0;
   double angle = heading;
   double target = (((((angle / 180) * M_PI) * radiusCenter) /2)*driveTicksPerInch); //convert to radians, multiply by the radius for arc length, divide by two, cause you're moving both sides in opposite directions
   pros::lcd::print(0, "target is %f", target);
   double error = target - (rightMeasured + leftMeasured)/2;
   double integral = 0;
   double lastError = 0;
   double integralCap = 200;
   double integralMin = 10;//prevention of integral windup
   while(abs(error) > 0){
       pros::lcd::print(5, "entered loop, %f", error);
       integral += error;
       if(integral > integralCap){//windup
           integral = integralCap;
       }
       if(integral > integralMin){
           integral = 0;
       }
       double out = (error * kP) + (integral * kI) + (lastError * kP);
       lastError = error;
       pros::lcd::print(0, "integral = %f", integral);
       pros::lcd::print(1, "error = %f", error);
       pros::lcd::print(2, "target = %f", target);
       pros::lcd::print(4, "output = %f", out);
       RB_MOTOR.move_absolute(-target,out);
       RM_MOTOR.move_absolute(-target,out);
       RF_MOTOR.move_absolute(-target,out);
       LF_MOTOR.move_absolute(target,out);
       LM_MOTOR.move_absolute(target,out);
       LB_MOTOR.move_absolute(target,out);
       pros::lcd::print(6, "stopped motors");
       rightMeasured = ((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
       leftMeasured = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
       lastError = error;
       error = target - (rightMeasured + leftMeasured)/2;
   }
   LF_MOTOR.move_velocity(0);
   LM_MOTOR.move_velocity(0);
   LB_MOTOR.move_velocity(0);
   RB_MOTOR.move_velocity(0);
   RM_MOTOR.move_velocity(0);
   RF_MOTOR.move_velocity(0);
}


void move(double distance) {
   LF_MOTOR.tare_position();
   LM_MOTOR.tare_position();
   LB_MOTOR.tare_position();
   RB_MOTOR.tare_position();
   RM_MOTOR.tare_position();
   RF_MOTOR.tare_position();


   distance *= driveTicksPerInch;


   double integral = 10;
   double lastError = distance;


   int restedStates = 0;
   int stalledStates = 0;
   double rightMeasured = ((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
   double leftMeasured = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
   double error = 0;
   /*while (restedStates < 5 && stalledStates < 50) {//amigo code
       double error = (distance - (rightMeasured + leftMeasured)/2);
       if (fabs(error) < 2.5*driveTicksPerInch) integral += error;
       if (fabs(error) < 0.05*driveTicksPerInch) integral = 0;


       if (fabs(error - lastError) < 0.05*driveTicksPerInch) stalledStates++;
       else stalledStates = 0;


       double out = kP * error + kI * integral + kD * (error - lastError);
       RB_MOTOR.move_voltage(out);
       RM_MOTOR.move_voltage(out);
       RF_MOTOR.move_voltage(out);
       LF_MOTOR.move_voltage(out);
       LM_MOTOR.move_voltage(out);
       LB_MOTOR.move_voltage(out);


       lastError = error;


       pros::delay(10);


       if (fabs(distance - (leftMeasured+ rightMeasured)/2) < 0.1*driveTicksPerInch) restedStates++;
       else restedStates = 0;
   }*/
   stopMotors();
}


void move_straight(double distance) {
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
   stopMotors();*/
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
}


void RunIntake(double target){
   intake.tare_position();
   target = 0;
   while(intake.get_position() >= 0){


   }
}


void stopMotors(){
   LF_MOTOR.move_voltage(0);
   LM_MOTOR.move_voltage(0);
   LB_MOTOR.move_voltage(0);
   RB_MOTOR.move_voltage(0);
   RM_MOTOR.move_voltage(0);
   RF_MOTOR.move_voltage(0);
}
