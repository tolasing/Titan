/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
/*
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PH6  //pin 2
  #define LEFT_ENC_PIN_B PH5  //pin 3
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
*/
#define LEFT_ENC_PIN_A 18
#define RIGHT_ENC_PIN_A 20
#define LEFT_ENC_PIN_B  8
#define RIGHT_ENC_PIN_B 6

const int windowSize = 5;  // Number of previous readings to consider
int leftRawReadings[windowSize];
int rightRawReadings[windowSize];
int leftReadIndex = 0;
int rightReadIndex = 0;
int leftRawSum=0;
int rightRawSum=0;


#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void pulse_count_left(int i);
void pulse_count_right(int i);
void pulse_count_left_wrapper_forward();
void pulse_count_left_wrapper_backward();
void clearLeftRawReadings();
void clearRightRawReadings();

