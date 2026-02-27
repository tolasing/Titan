/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 12
  #define LEFT_MOTOR_BACKWARD  1
  #define RIGHT_MOTOR_FORWARD  11
  #define LEFT_MOTOR_FORWARD   9
  #define RIGHT_MOTOR_ENABLE 13  
  #define LEFT_MOTOR_ENABLE 13
  #define LIMIT_UP 7
  #define LIMIT_DOWN 8
#endif

void initMotorController();
void setMotorSpeed(int i, double spd);
void setMotorSpeeds(double leftSpeed, double rightSpeed);
