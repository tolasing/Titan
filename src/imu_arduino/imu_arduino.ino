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
  #include "I2Cdev.h"
  #include "MPU6050_6Axis_MotionApps20.h"
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
MPU6050 mpu;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
  mpuInterrupt = true;
}

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

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void read_imu(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float yaw,float pitch,float roll) {
  float accelX = ax/16384.0 * 9.8;
  float accelY = ay / 16384.0 * 9.8;
  float accelz = az / 16384.0 * 9.8;

  float velX = gx * (PI/180);
  float velY = gy * (PI/180);
  float velZ = gz * (PI/180);

  Serial.print(accelX);
  Serial.print(" ");
  Serial.print(accelY);
  Serial.print(" ");
  Serial.print(accelz);
  Serial.print(" ");
  Serial.print(velX);
  Serial.print(" ");
  Serial.print(velY);
  Serial.print(" ");
  Serial.print(velZ);
  Serial.print(" ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);
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
int runCommand(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float yaw,float pitch,float roll) {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atof(argv1);
  arg2 = atof(argv2);

  switch (cmd) {
#ifdef USE_BASE
    case READ_IMU:
      read_imu(ax,ay,az,gx,gy,gz,yaw,pitch,roll);
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
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(BAUDRATE);
  pinMode(INTERRUPT_PIN, INPUT);
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.CalibrateGyro();
  mpu.setXAccelOffset(-4633);
  mpu.setYAccelOffset(9);
  mpu.setZAccelOffset(1447);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually, the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  Serial.println("Invalid Command");
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t gyroX, gyroY, gyroZ;
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   // Serial.print("ypr\t");
    //Serial.print(ypr[0] * 180 / M_PI);
    //Serial.print("\t");
    //Serial.print(ypr[1] * 180 / M_PI);
    //Serial.print("\t");
    //Serial.println(ypr[2] * 180 / M_PI);

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand(ax,ay,az,gx,gy,gz,ypr[0],ypr[1],ypr[2]);
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
