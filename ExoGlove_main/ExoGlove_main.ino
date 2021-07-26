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

#define fsrThumb   36   // Thumb fsr pin for force feedback
#define fsrIndex   39   // Index fsr pin for force feedback
#define fsrMiddle  34   // Middle fsr pin for force feedback
#define fsrRingP   35   // Ring & Pinky fsr pin for force feedback
#define AN_Pot1    33   // Thumb pot pin for position feedback
#define AN_Pot2    25   // Index pot pin for position feedback
#define AN_Pot3    26   // Middle pot pin for position feedback
#define AN_Pot4    27   // Ring & pinky pot pin for position feedback, 34-39 are input only
#define Thumbpin   14   // Thumb servo pin for actuation
#define Indexpin   12   // Index servo pin for actuation
#define Middlepin  13   // Middle servo pin for actuation
#define RingPPin   21   // Ring & Pinky servo pin for actuation

#define FILTER_LEN 20   // length of 20 points, longer it is, smoother filter but takes longer time

uint32_t AN_Pot_Buffer[FILTER_LEN] = {0}; // empty buffer
int AN_Pot_i = 0;                         // buffer index

int AN_Pot1_Raw, AN_Pot2_Raw, AN_Pot3_Raw, AN_Pot4_Raw;  // variables to hold value read from the analog pin
int AN_Pot1_Filtered = 0; 
int AN_Pot2_Filtered = 0; 
int AN_Pot3_Filtered = 0; 
int AN_Pot4_Filtered = 0; // filtered

float pot1Voltage = 0.0; float pot1Final = 0.0;
float pot2Voltage = 0.0; float pot2Final = 0.0;
float pot3Voltage = 0.0; float pot3Final = 0.0;
float pot4Voltage = 0.0; float pot4Final = 0.0;

float  angle1, anglesec, angle2, angle3, angle4;   // variable to hold the angle1 for the servo motor (500-2500)usec

void setup() {
  Serial.begin(115200); // open a serial connection to your computer
  delay(1000);          // allow some time to set up serial monitor
  // Setup ADC
  
  // attaches the servo on pins to the servo objects
  ThumbServo.attach(Thumbpin); // attaches the servo on GPIO14 to the servo object
  IndexServo.attach(Indexpin); // attaches the servo on GPIO25 to the servo object
  MiddleServo.attach(Middlepin); // attaches the servo on GPIO26 to the servo object
  RandPServo.attach(Ringpin); // attaches the servo on GPIO27 to the servo object
}

void loop() {
  //an  gle1= 500;
  AN_Pot1_Raw = analogRead(AN_Pot1); // read the value of the potentiometer
  AN_Pot2_Raw = analogRead(AN_Pot2); // read the value of the potentiometer
  AN_Pot3_Raw = analogRead(AN_Pot3); // read the value of the potentiometer
  AN_Pot4_Raw = analogRead(AN_Pot4); // read the value of the potentiometer

  AN_Pot1_Filtered = readADC_Avg(AN_Pot1_Raw); // go through filtering
  pot1Voltage = readADC_Cal(AN_Pot1_Filtered);     // calibrate it.
  pot1Final = Voltage_Cal(pot1Voltage);  // Trendline equation from data collected comparing measurements from DMM (V) Vs ESP32 (V)

  AN_Pot2_Filtered = readADC_Avg(AN_Pot2_Raw); // go through filtering
  pot2Voltage = readADC_Cal(AN_Pot2_Filtered);     // calibrate it.
  pot2Final = Voltage_Cal(pot2Voltage);

  AN_Pot3_Filtered = readADC_Avg(AN_Pot3_Raw); // go through filtering
  pot1Voltage = readADC_Cal(AN_Pot3_Filtered);     // calibrate it.
  pot3Final = Voltage_Cal(pot3Voltage);

  AN_Pot4_Filtered = readADC_Avg(AN_Pot4_Raw); // go through filtering
  pot4Voltage = readADC_Cal(AN_Pot4_Filtered);     // calibrate it.
  pot4Final = Voltage_Cal(pot4Voltage);
  
  // scale the numbers from the pot
  angle1= map(pot1Final, 0.96, 3.07, 0, 300); // the values 0.96 and 3.07 came from linear range (0.96-3.07V) of ADC 
  angle2= map(pot2Final, 0.96, 3.07, 0, 300);
  angle3= map(pot3Final, 0.96, 3.07, 0, 300);
  angle4= map(pot4Final, 0.96, 3.07, 0, 300);
  // map degrees into duty cycles (in microsec)
  // anglesec = map(potVal1, 95, 1145, 500, 2500); // only for testing purposes
  
  // print out the value to the Serial Monitor
  Serial.print("pot1Final: "); Serial.print(pot1Final);
  // print out the value to the Serial Monitor
  Serial.print(", pot2Final: "); Serial.print(pot2Final);
  // print out the angle set for the servo motor
  Serial.print(", angle1: "); Serial.print(angle1);
  // print out the measured angle of the servo motor
  Serial.print(", angle2: "); Serial.print(angle2);
   // print out the angle set for the servo motor

  // set the servo position
  IndexServo.writeMicroseconds(anglesec);
  // wait for the servo to get there
  delay(150); 

}

uint32_t readADC_Cal(int ADC_Raw)  // ADC calibration
{
  esp_adc_cal_characteristics_t adc_chars;

  // ADC1, 11dB (0-4095), 12 bits, default Vref = 1.1V
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars); // address of adc_chars
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

uint32_t readADC_Avg(int ADC_Raw) //moving average filter
{
  int i = 0;
  uint32_t Sum = 0;
  
  AN_Pot_Buffer[AN_Pot_i++] = ADC_Raw;
  if(AN_Pot_i == FILTER_LEN) // if buffer index full, then reset index
  {
    AN_Pot_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {
    Sum += AN_Pot_Buffer[i];
  }
  return (Sum/FILTER_LEN);
}

float Voltage_Cal(float FilteredVolt) // takes float type input (in mV)
{
  float FinalVolt = 0;
  return FinalVolt = 0.9955*FilteredVolt/1000 - 0.0535; //trendline equation obtained from data collected comparing measurements from DMM (V) Vs ESP32 (V) 
}
