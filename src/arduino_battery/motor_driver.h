/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 4
  #define LEFT_MOTOR_BACKWARD  5
  #define RIGHT_MOTOR_FORWARD  3
  #define LEFT_MOTOR_FORWARD   6
  #define RIGHT_MOTOR_ENABLE 2  
  #define LEFT_MOTOR_ENABLE 7
#endif

void initMotorController();
void setMotorSpeed(int i, double spd);
void setMotorSpeeds(double leftSpeed, double rightSpeed);
