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


void turn(double heading, double Kp, double Kd, double Ki, double O, double U) { //turns a certain amount of degrees
/*New turn code with IMU*/
IMU.set_rotation(0);
double error = heading;
error *= O;
error /= U;
    if(fabs(error) > 180){
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

        double power = error * Kp + integral * Ki + (error-prevError) * Kd;
        moveRight(-1*power);
        moveLeft(power);

        prevError = error;
        pros::delay(10);
        if(abs(error)< 1){
            break;
        }
        if(fabs(prevError)-fabs(error)<0.05 && fabs(error)<0.4){
            break;
        }
    }
   stopMotors();
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

void move(double distance, double kP, double kI, double kD) {
    tareMotors();
   double rightOutput = 0.0;
   double leftOutput = 0.0;
   distance *= driveTicksPerInch;
   distance /= 24.0;
   double target = distance;
   double integral = 0.0;
   double rightMeasured = ((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
   double leftMeasured = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
   double leftVelocity = ((LB_MOTOR.get_actual_velocity() + LF_MOTOR.get_actual_velocity() + LM_MOTOR.get_actual_velocity())/3);
   double rightVelocity = ((RB_MOTOR.get_actual_velocity() + RF_MOTOR.get_actual_velocity() + RM_MOTOR.get_actual_velocity())/3);//average right velocity in rpms
   double error = 0.0;//error between the two sides
   double error2 = 0.0;//error between real velocities and fake velocities
   double distanceT = 0.0;//area under the curve
   double distanceT2 = 0.0;//actual position in ticks
   rightVelocity * rpmToTps;
   leftVelocity * rpmToTps;
   while(target > distanceT2){
    rightVelocity = rightVelocity * 0.01;//how much time passed since last taking of velocity, then multiply by seconds passed to get ticks traveled
    leftVelocity = leftVelocity* 0.01;
    if(rightVelocity > leftVelocity){
        error = rightVelocity - leftVelocity;
    }
    distanceT += ((rightVelocity + leftVelocity)/2.0);//better way to calculate distance traveled?
    distanceT2 = (rightMeasured + leftMeasured)/2.0;
    rightOutput = ((target - distanceT)*kI - (error * kD) - (distanceT-distanceT2)*kP);//missing length left in ticks 
    leftOutput = ((target - distanceT)*kI + (error * kD) - ((distanceT-distanceT2)*kP));
    pros::lcd::print(0, "before calling moveRight");
    moveRight(rightOutput);
    pros::lcd::print(0, "after calling moveRight");
    moveLeft(leftOutput);
    pros::lcd::print(0, "right output %f", rightOutput);
    rightMeasured = ((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
    leftMeasured = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
    leftVelocity = ((LB_MOTOR.get_actual_velocity() + LF_MOTOR.get_actual_velocity() + LM_MOTOR.get_actual_velocity())/3);
    rightVelocity = ((RB_MOTOR.get_actual_velocity() + RF_MOTOR.get_actual_velocity() + RM_MOTOR.get_actual_velocity())/3);//average right velocity in rpms
    pros::delay(10);
   }
   stopMotors();//hit the ideal distance so stop yourself
}
void moveBack(double distance, double kP, double kI, double kD) {
    tareMotors();
   double rightOutput = 0.0;
   double leftOutput = 0.0;
   distance *= driveTicksPerInch;
   distance /= 24.0;
   double target = -distance;
   double integral = 0.0;
   double rightMeasured = ((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
   double leftMeasured = ((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
   double leftVelocity = ((LB_MOTOR.get_actual_velocity() + LF_MOTOR.get_actual_velocity() + LM_MOTOR.get_actual_velocity())/3);
   double rightVelocity = ((RB_MOTOR.get_actual_velocity() + RF_MOTOR.get_actual_velocity() + RM_MOTOR.get_actual_velocity())/3);//average right velocity in rpms
   double error = 0.0;//error between the two sides
   double error2 = 0.0;//error between real velocities and fake velocities
   double distanceT = 0.0;//area under the curve
   double distanceT2 = 0.0;//actual position in ticks
   rightVelocity * rpmToTps;
   leftVelocity * rpmToTps;
   while(target < distanceT2){
    rightVelocity = rightVelocity * 0.01;//how much time passed since last taking of velocity, then multiply by seconds passed to get ticks traveled
    leftVelocity = leftVelocity* 0.01;
    if(rightVelocity > leftVelocity){
        error = rightVelocity - leftVelocity;
    }
    distanceT -= ((rightVelocity + leftVelocity)/2.0);//better way to calculate distance traveled?
    distanceT2 = -(rightMeasured + leftMeasured)/2.0;
    rightOutput = ((target - distanceT)*kI - (error * kD) - (distanceT-distanceT2)*kP);//missing length left in ticks 
    leftOutput = ((target - distanceT)*kI + (error * kD) - ((distanceT-distanceT2)*kP));
    pros::lcd::print(0, "before calling moveRight");
    moveRight(rightOutput);
    pros::lcd::print(0, "after calling moveRight");
    moveLeft(leftOutput);
    pros::lcd::print(0, "right output %f", rightOutput);
    rightMeasured = -((RB_MOTOR.get_position() + RF_MOTOR.get_position() + RM_MOTOR.get_position())/3);
    leftMeasured = -((LB_MOTOR.get_position() + LF_MOTOR.get_position() + LM_MOTOR.get_position())/3);
    leftVelocity = -((LB_MOTOR.get_actual_velocity() + LF_MOTOR.get_actual_velocity() + LM_MOTOR.get_actual_velocity())/3);
    rightVelocity = -((RB_MOTOR.get_actual_velocity() + RF_MOTOR.get_actual_velocity() + RM_MOTOR.get_actual_velocity())/3);//average right velocity in rpms
    pros::delay(10);
   }
   stopMotors();//hit the ideal distance so stop yourself
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
   intake.move_velocity(600);
   //intake.move_absolute(3200,200);
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