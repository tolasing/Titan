/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/


  #define RIGHT_MOTOR_BACKWARD 6
  #define LEFT_MOTOR_BACKWARD  10
  #define RIGHT_MOTOR_FORWARD  5
  #define LEFT_MOTOR_FORWARD   9
  #define RIGHT_MOTOR_ENABLE 12  
  #define LEFT_MOTOR_ENABLE 13


void initMotorController();
void setMotorSpeed(int i, double spd);
void setMotorSpeeds(double leftSpeed, double rightSpeed);
