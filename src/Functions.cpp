#include "Functions.h"
#include "MotorInit.h"
#include "PIDControls.h"
#include "main.h"
void autonSelector() {
   //loops through autonomous options until you pick a desired one. If you loop thru all, it will repeat
   int selected = 1;
   while(selected < amountOfAuton){
   }
}
void slowIntake(){
   if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      intake.move_velocity(300);
   }
   else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      intake.move_velocity(-500);
   }
}

void driveFunc(double power, double turn) {
   //puts controls into a cubed function to avoid jerk(lurch)
   double left = power + (turn * 0.6);
   double right = power -(turn * 0.6);
   double leftCubed = (400*left*left*left);
   double rightCubed = (400*right*right*right);
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

/*void clampTeleOP(){
   if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
      if(mode = true){
         clampDown();
         mode = false;
      }
      else{
         clampRelease();
         mode = true;
      }
   }
}*/
void sigmaFlipOut185(){
   if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
      if(mode2 == true){
         flipOut.set_value(1);
         mode2 = false;
      }
      else{
         flipOut.set_value(0);
         mode2 = true;
         }
   }
}
void clampDown() {
   clamper.set_value(1);
}


void clampRelease() {
   clamper.set_value(0);
}



/*void driveIntake() {
   intakeRunner.move_velocity(140);//runs the intake at the desired volociy
   double start = 0.0;
   double oneRotation = 0.0;
   Intake.set_position(start - (Intake.get_position()%oneRotation));
  
}*/
void moveIntake(){
   if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
      if(mode3 == true){
         intake.move_velocity(500);
         mode3 = false;
      }
      else{
         intake.move_velocity(500);
         intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
         mode3 = true;
         }
   }
}
void moveArm(){
   if(master.get_digital(DIGITAL_L1)){
       Arm.move_velocity(200);
   }
   else if(master.get_digital(DIGITAL_L2)){
       Arm.move_velocity(-200);
   }
   else{
      Arm.move_velocity(0);
      Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
   }

}