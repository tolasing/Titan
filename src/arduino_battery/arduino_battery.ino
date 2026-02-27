#define USE_BASE

#ifdef USE_BASE
  #define ARDUINO_ENC_COUNTER
  /* L298 Motor driver */
  #define L298_MOTOR_DRIVER
#endif

/* Serial port baud rate */
#define BAUDRATE     9600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #include "string.h"
  #include <Wire.h>
 
#else
#endif

/* Include definition of serial commands */
#include "commands.h"

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"
#endif

/* Variable initialization */
const int sensorPin = A0;
// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0; 
float in_voltage=0.0;
float average_voltage=0.0;
int count =0;
int sensorValue=0;
float voltage=0.0;



// A pair of variables to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
double arg1;
double arg2;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;


void read_battery(float average_voltage) {
  Serial.println(average_voltage, 4);
 
  
}

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand(float average_voltage) {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atof(argv1);
  arg2 = atof(argv2);

  switch (cmd) {
#ifdef USE_BASE
    case READ_BATTERY:
      read_battery(average_voltage);
      break;
    case RESET_ENCODERS:
      resetEncoders();
      Serial.println("OK");
      break;
#endif
    default:
      Serial.println("Invalid Command");
      break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(9600);
}

void loop() {

  while(count<5)
  {
  // Read the analog voltage from the sensor
   sensorValue = analogRead(sensorPin);

  // Convert the analog value to voltage (assuming a 5V reference)
   voltage = (sensorValue *5.024 )/ 1024.0;
 
  // Calculate voltage at divider input
   //in_voltage = voltage / (R2/(R1+R2)) ; 
   in_voltage+=voltage/0.197;
   count++;

  }

  count=0;
  average_voltage=in_voltage/5;
  in_voltage=0.0;




  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand(average_voltage);
      average_voltage=0.0;
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
}
