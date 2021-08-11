/*
ExoGlove source code
  Parts required:
  - servo motor x4
  - 10 kilohm potentiometer x4
July 7, 2021
by Ananda Aw, Noah Stoffel
*/

// include the Servo library
#include "ESP32Servo.h"
#include "esp_adc_cal.h"

Servo ThumbServo;  // Servo that controls thumb
Servo IndexServo;  // Servo that controls index finger
Servo MiddleServo;  // Servo that controls middle finger
Servo RandPServo;  // Servo that controls ring and pinky fingers

#define fsrThumb   34   // Thumb fsr pin for force feedback
#define fsrIndex   35   // Index fsr pin for force feedback
#define fsrMiddle  36   // Middle fsr pin for force feedback
#define fsrRingP   39   // Ring & Pinky fsr pin for force feedback
#define ENC1       32   // Thumb pot pin for position feedback 32
#define ENC2       33   // Index pot pin for position feedback 33
#define ENC3       25   // Middle pot pin for position feedback 25
#define ENC4       26   // Ring & pinky pot pin for position feedback, 34-39 are input only 26
#define controlpin 27   // Pot pin for thumb servo control
#define IndexCtrl  14   // Pot pin for index servo control
#define MiddleCtrl 12   // Pot pin for middle servo control
#define RandPCtrl  13   // Pot pin for ring and pinky servo control
#define Thumbpin   21   // Thumb servo pin for actuation 27
#define Indexpin   19   // Index servo pin for actuation 14
#define Middlepin  18   // Middle servo pin for actuation 12
#define RingPpin   5    // Ring & Pinky servo pin for actuation 13
#define button     15
#define LEDpin     2

//#define FILTER_LEN 20   // length of 20 points, longer it is, smoother filter but takes longer time

int control, control_filtered, angleToTurn, ENC1_Raw, ENC2_Raw, ENC3_Raw, ENC4_Raw;  // variables to hold value read from the analog pins
int fsr1_Raw, fsr2_Raw, fsr3_Raw, fsr4_Raw; // to hold analog read from fsr
int control2, control3, control4; // variables to hold values for potentiometer control
int IndexTurn, MiddleTurn, RandPTurn;      // variables to hold values for turning. 
int Thumbpos, Indexpos, Middlepos, RandPpos; // variables to hold the angle positions from position pots 

#define filter_len 21   // kaiser filter length
int filtered1, filtered2, filtered3, filtered4; // this holds filtered values from encoders
int fsrfilt1, fsrfilt2, fsrfilt3, fsrfilt4;     // this holds filtered values from fsr.
float tempenc1_1[filter_len], tempenc1_2[filter_len], tempenc2_1[filter_len], tempenc2_2[filter_len]; // temp arrays to hold 21 filtered values
float tempenc3_1[filter_len], tempenc3_2[filter_len], tempenc4_1[filter_len], tempenc4_2[filter_len];
float tempfsr1_1[filter_len], tempfsr1_2[filter_len], tempfsr2_1[filter_len], tempfsr2_2[filter_len]; // fsr temp arrays for filtering.
float tempfsr3_1[filter_len], tempfsr3_2[filter_len], tempfsr4_1[filter_len], tempfsr4_2[filter_len];

float kernelcoef[] = {0.0146, 0.0217,  0.0293, 0.0371, 0.0450, 0.0524, 0.0590, 0.0646, 0.0688, 0.0714, 0.0723, // cutoff freq of 4Hz
                        0.0714, 0.0688, 0.0646, 0.0590, 0.0524, 0.0450, 0.0371, 0.0293, 0.0217, 0.0146};

volatile unsigned int counter = 0;

void IRAM_ATTR Move_Servo() // interrupt handler
{
  static unsigned long last_interrupt_time = 0; // debounce section
  unsigned long interrupt_time = millis();
  
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if ((interrupt_time - last_interrupt_time) > 200) 
  {
    // Toggle The LED
    digitalWrite(LEDpin, !digitalRead(LEDpin));
    
    if(counter < 3) 
    {
      counter++;
    }
    else          // reset counter if greater than 3
    {
      counter = 0;
    }
    
  }
  last_interrupt_time = interrupt_time;
}

void setup() {
  Serial.begin(115200); // open a serial connection to your computer
  delay(100);          // allow some time to set up serial monitor
  // Setup ADC
  analogReadResolution(12); // set reading resolution to 12 bits, 0-4095.
  
  // attaches the servo on pins to the servo objects
  ThumbServo.attach(Thumbpin); // attaches the servo on GPIO14 to the servo object
  IndexServo.attach(Indexpin); // attaches the servo on GPIO25 to the servo object
  MiddleServo.attach(Middlepin); // attaches the servo on GPIO26 to the servo object
  RandPServo.attach(RingPpin); // attaches the servo on GPIO27 to the servo object

  ThumbServo.writeMicroseconds(500); // set initial position to 0 deg
  IndexServo.writeMicroseconds(500);
  MiddleServo.writeMicroseconds(500);
  RandPServo.writeMicroseconds(500);
   
  pinMode(LEDpin, OUTPUT);
  pinMode(button, INPUT);
  attachInterrupt(digitalPinToInterrupt(button), Move_Servo, FALLING); 
}

void loop() {
  ENC1_Raw = analogRead(ENC1); // read position of motors
  ENC2_Raw = analogRead(ENC2);
  ENC3_Raw = analogRead(ENC3);
  ENC4_Raw = analogRead(ENC4);
  
  filtered1 = conv_rt(ENC1_Raw, tempenc1_1, tempenc1_2); // filter
  filtered2 = conv_rt(ENC2_Raw, tempenc2_1, tempenc2_2);
  filtered3 = conv_rt(ENC3_Raw, tempenc3_1, tempenc3_2);
  filtered4 = conv_rt(ENC4_Raw, tempenc4_1, tempenc4_2);

  Thumbpos = map(filtered1, 0, 4095, 0, 333.3);
  Indexpos = map(filtered2, 0, 4095, 0, 333.3);
  Middlepos = map(filtered3, 0, 4095, 0, 333.3);
  RandPpos = map(filtered4, 0, 4095, 0, 333.3);

  fsr1_Raw = analogRead(fsrThumb);
  fsr2_Raw = analogRead(fsrIndex);
  fsr3_Raw = analogRead(fsrMiddle);
  fsr4_Raw = analogRead(fsrRingP);

  fsrfilt1 = conv_rt(fsr1_Raw, tempfsr1_1, tempfsr1_2); // filter
  fsrfilt2 = conv_rt(fsr2_Raw, tempfsr2_1, tempfsr2_2);
  fsrfilt3 = conv_rt(fsr3_Raw, tempfsr3_1, tempfsr3_2);
  fsrfilt4 = conv_rt(fsr4_Raw, tempfsr4_1, tempfsr4_2);
  
  //Serial.print(control); Serial.print(", ");
  Serial.print(counter); Serial.print(", ");  
  //Serial.print(angleToTurn); Serial.print(", ");
  //Serial.print(ENC1_Raw); Serial.print(", ");
  //Serial.print(filtered1); Serial.print(", ");
  //Serial.print(Thumbpos); Serial.print(", ");
  //Serial.print(filtered2); Serial.print(", ");
  //Serial.print(Indexpos); Serial.println(", ");
  //Serial.print(Middlepos); Serial.print(", ");
  Serial.print(RandPpos); Serial.println(", ");
  
  control = analogRead(controlpin);
  angleToTurn = map(control, 0, 4095, 500, 2500); // convert to microseconds
    
  if(counter == 1) // when odd
  {
    ThumbServo.writeMicroseconds(2000); // turn 202.5 deg when counter is even
    IndexServo.writeMicroseconds(2000);
    MiddleServo.writeMicroseconds(2000);
    RandPServo.writeMicroseconds(2000);
    delay(100);
  }
  else if(counter == 2) // pot control mode
  {
    ThumbServo.writeMicroseconds(angleToTurn); // turn the motors, Thumb at pot = 0 is open
    IndexServo.writeMicroseconds(angleToTurn);
    MiddleServo.writeMicroseconds(angleToTurn); // middle at pot = 0 is closed, reversed
    RandPServo.writeMicroseconds(angleToTurn);

    delay(100); // delay a little
    
    
  }
  else if(counter == 3) // each pot controls different servos
  {
    control2 = analogRead(IndexCtrl); // read pot value to set motor control
    control3 = analogRead(MiddleCtrl);
    control4 = analogRead(RandPCtrl);

    IndexTurn = map(control2, 0, 4095, 500, 2500); // convert to duty cycle, index inverted
    MiddleTurn = map(control3, 0, 4095, 500, 2500);
    RandPTurn = map(control4, 0, 4095, 500, 2500);

    ThumbServo.writeMicroseconds(angleToTurn); // turn the motors individually
    IndexServo.writeMicroseconds(IndexTurn);
    MiddleServo.writeMicroseconds(MiddleTurn);
    RandPServo.writeMicroseconds(RandPTurn);
  }

  else
  {
    ThumbServo.writeMicroseconds(500); // reset the motors
    IndexServo.writeMicroseconds(500);
    MiddleServo.writeMicroseconds(500);
    RandPServo.writeMicroseconds(500);
    delay(150);
  }
}

float conv_rt(int input, float temp[], float temp2[])
{
 float output = input*kernelcoef[0];
 
 temp2[0] = input;

 for(int i = 0; i<(filter_len-1); i++)
 {
  output += temp[i]*kernelcoef[i+1];
  temp2[i+1] = temp[i];
 }
 
 for(int j =0; j<filter_len; j++)
 {
  temp[j] = temp2[j];
 }
 return output;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*uint32_t readADC_Avg(int ADC_Raw, uint32_t buffer1[]) //moving average filter
{
  int i = 0;
  uint32_t Sum = 0;
  int ENC_i = 0;
  
  buffer1[ENC_i++] = ADC_Raw;
  
  if(ENC_i == FILTER_LEN) // if buffer index full, then reset index
  {
    ENC_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {
    Sum += buffer1[i];
  }
  return (Sum/FILTER_LEN);
}*/ 
