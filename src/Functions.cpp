#include "Functions.h"
#include "MotorInit.h"
#include "PIDControls.h"
void autonSelector() {
   //loops through autonomous options until you pick a desired one. If you loop thru all, it will repeat
   int selected = 1;
   while(selected < amountOfAuton){
   }
}


void driveFunc(double power, double turn) {
   //puts controls into a cubed function to avoid jerk(lurch)
   double left = power + (turn * 0.6);
   double right = power -(turn * 0.6);
   double leftCubed = (600*left*left*left);
   double rightCubed = (600*right*right*right);
   leftCubed /= (127*127*127);
   rightCubed /= (127*127*127);
   if(abs(rightCubed) >= 500){


   }
   RB_MOTOR.move_velocity(rightCubed);
   RM_MOTOR.move_velocity(rightCubed);
   RF_MOTOR.move_velocity(rightCubed);
   LB_MOTOR.move_velocity(leftCubed);
   LM_MOTOR.move_velocity(leftCubed);
   LF_MOTOR.move_velocity(leftCubed);
}


void clampDown() {
   clamper.set_value(false);
}


void clampRelease() {
   clamper.set_value(true);
}


/*void driveIntake() {
   intakeRunner.move_velocity(140);//runs the intake at the desired volociy
   double start = 0.0;
   double oneRotation = 0.0;
   Intake.set_position(start - (Intake.get_position()%oneRotation));
  
}*/
void moveIntake(){
   if(master.get_digital(DIGITAL_R1)){
       intake.move_velocity(200);
   }
   else if(master.get_digital(DIGITAL_R2)){
       intake.move_velocity(-200);
   }
   else{
       intake.move_velocity(0);
   }
}
void moveArm(){
   if(master.get_digital(DIGITAL_L1)){
       Arm.move_absolute(300,200);
   }
   else{
       Arm.move_absolute(0,200);
   }
}
