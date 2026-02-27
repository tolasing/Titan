/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/
  void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, double spd) {
    unsigned char reverse = 0;
    
    //if(spd>0&&spd<90){spd=90;}

    if (spd < 0)
    {
      spd = -spd; 
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == LEFT) { 
      if      (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if      (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD, spd*0.90); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
    }
  }
  
  void setMotorSpeeds(double leftSpeed, double rightSpeed) {
    /*
    if(leftSpeed ==0&&rightSpeed==0)
    {
      //detachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A));
    //detachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A));
    }*/
    setMotorSpeed(RIGHT, rightSpeed);
    setMotorSpeed(LEFT, leftSpeed);
  }

