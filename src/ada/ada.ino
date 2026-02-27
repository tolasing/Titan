#define USE_BASE

#ifdef USE_BASE
  #define ARDUINO_ENC_COUNTER
  /* L298 Motor driver */
  #define L298_MOTOR_DRIVER
#endif

/* Serial port baud rate */
#define BAUDRATE     115200

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #include "string.h"
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BNO055.h>
  #include <utility/imumaths.h>
 
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
// indicates whether MPU interrupt pin has gone high


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


/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


void read_imu(float acc_x,float gyro_z,float quat_z,float quat_w) {
  Serial.print(acc_x, 4);
  Serial.print(" ");
  Serial.print(gyro_z, 4);
  Serial.print(" ");
  Serial.print(quat_z, 4);
  Serial.print(" ");
  Serial.println(quat_w, 4);
  
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
int runCommand(float acc_x,float gyro_z,float quat_z,float quat_w) {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atof(argv1);
  arg2 = atof(argv2);

  switch (cmd) {
#ifdef USE_BASE
    case READ_IMU:
      read_imu(acc_x,gyro_z,quat_z,quat_w);
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
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  Serial.println("Invalid Command");
}

void loop() {
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> accer=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyer=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  float acc_x,gyro_z,quat_z,quat_w;

  acc_x=accer.x();
  gyro_z=gyer.z();
  quat_z=quat.z();
  quat_w=quat.w();
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand(acc_x,gyro_z,quat_z,quat_w);
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
