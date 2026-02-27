
#include "commands.h"
// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Limit Switch connections
int limit_down = 5;
int limit_up = 4;

double pallet_position=0.0;
double limit_up_state;
double limit_down_state;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

#define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;

int runCommand() {

  switch(cmd) {
case MOTOR_DOWN:
  lastMotorCommand = millis();
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 70);
  //Serial.println("down");
  break;

case MOTOR_UP:
  lastMotorCommand = millis();
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA,70);
  //Serial.println("up"); 
  break;

case READ_ACTUATOR:
Serial.print(pallet_position);
Serial.print("\n");
break;

case STOP_MOTOR:
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA,0);
break;
    
default:
  Serial.println("Invalid Command");
  break;
      }
}

void resetCommand() {
  cmd = NULL;
}


void setup() {
  Serial.begin(115200);
	// Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
  pinMode(limit_up,INPUT_PULLUP);
  pinMode(limit_down,INPUT_PULLUP);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
}

void loop() {

  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    // The first arg is the single-letter command
    cmd = chr;
    }

    // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
   // detachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A));
    //detachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A));
    analogWrite(enA,0);
  }
	  check_limit_switch();
}

// Check the status of the limit switch and control the motor accordingly.
void check_limit_switch() {
  limit_up_state=digitalRead(limit_up);
  limit_down_state=digitalRead(limit_down);
  if (limit_up_state==LOW)
  {
    analogWrite(enA,0);
    pallet_position=1.0;//if the pallet_loader is up mark it as 1.0
    //Serial.println("pallet loader is up");
  }

  else if(limit_down_state==LOW)
  { 
    analogWrite(enA,0);
    pallet_position=0.0;//if the pallet_loader is down mark it as 0.0
    //Serial.println("pallet loader is down");
  }
  

	// Set motors to maximum speed
	// For PWM maximum possible values are 0 to 25
  

}
	
