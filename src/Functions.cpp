#include "Functions.h"
#include "MotorInit.h"
#include "PIDControls.h"
#include "main.h"
void intake_fn(){
   double lastPosition = intake.get_position();
   while(true){
      if(intake.get_position() < lastPosition+600){
		intake.move_velocity(-200);
		pros::delay(50);
		intake.move_velocity(600);
      }
   }
}

void driveFunc(double power, double turn) {
   //puts controls into a cubed function to avoid jerk(lurch)
   double left = power + (turn);
   double right = power -(turn);
   double leftCubed = (600*left*left*left);
   double rightCubed = (600*right*right*right);
   leftCubed /= (127*127*127);
   rightCubed /= (127*127*127);
   if(power == 0){
      RB_MOTOR.move_velocity(turn);
      RM_MOTOR.move_velocity(turn);
      RF_MOTOR.move_velocity(turn);
      LF_MOTOR.move_velocity(-turn);
      LB_MOTOR.move_velocity(-turn);
      LM_MOTOR.move_velocity(-turn);
   }
   if(abs((rightCubed -prevPower)) > 600){

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



void driveIntake() {
   intake.move_velocity(130);
   if(DSensor.get() <= 148){
      intake.move_velocity(-50);
      pros::lcd::print(0, "Sensed");
      pros::delay(700);
   }
}
void driveIntakeUltimate() {
   intake.move_velocity(130);
   /*while(Optical.getcolor != green)
   if(DSensor.get() <= 145){
      intake.move_velocity(-50);
      pros::lcd::print(0, "Sensed");
      pros::delay(700);
   }*/
}
void moveIntake(){
   if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
      while(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
         
         intake.move_velocity(-300);
      }
      intake.move_velocity(0);
   }
   if(speed != 1 && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
         intake.move_velocity(500);
         speed = 1;
   }
   else if(speed !=0 && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
		intake.move_velocity(0);
      speed = 0;
	}
   else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
      while(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
         intake.move_velocity(50);
      }
      intake.move_velocity(0);
   }
   else if(speed != 0 && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
      intake.move_velocity(0);
      speed = 0;
   }
   pros::delay(50);
}
void moveIntakeSunny(){
   if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
      while(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
         intake.move_velocity(-300);
         driveFunc(((master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*0.75)+(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*0.25)),(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*0.25)+(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)*0.75));
      }
      intake.move_velocity(0);
   }
   if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
         while(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake.move_velocity(500);
            driveFunc(((master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*0.75)+(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*0.25)),(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*0.25)+(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)*0.75));
      }
      intake.move_velocity(0);
   }
   else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
      while(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
         intake.move_velocity(50);
         driveFunc(((master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*0.75)+(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*0.25)),(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*0.25)+(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)*0.75));
      }
      intake.move_velocity(0);
   }
   else if(speed != 0 && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
      intake.move_velocity(0);
      speed = 0;
   }
   pros::delay(50);
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
void clampTeleOP(){
   if(clampmode == true && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
      clampDown();
      clampmode = false;
   }
   else if(clampmode == false && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
      clampRelease();
      clampmode = true;
   }
}
void autonSelector(){
   int numAutons = 5;
   std::string autons[5]={"Red Left  ","Blue Left ","Red Right ", "Blue Right", "skills    "};
    while(master.get_digital_new_press(DIGITAL_A)==0){
        master.print(2,0,"valve");
        master.print(0,0,"%s", autons[autonSelected]);
        if(master.get_digital_new_press(DIGITAL_RIGHT)){
            master.clear();
            pros::delay(60);
            autonSelected = (autonSelected + 1 + numAutons) % numAutons;
        } else if (master.get_digital_new_press(DIGITAL_LEFT)){
            master.clear();
            pros::delay(60);
            autonSelected = (autonSelected - 1 + numAutons) % numAutons;
        }
        master.print(0,0,"%s", autons[autonSelected]);
    }
    master.rumble("----");
    pros::delay(100);
    master.clear();
    pros::delay(2000);
}
