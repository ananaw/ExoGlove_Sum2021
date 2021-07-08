/*
Testing of the 2000 series dual mode motor with cable, possibly with potentiometer
  Parts required:
  - servo motor
  - 10 kilohm potentiometer
June 29, 2021
by Ananda Aw
*/

// include the Servo library
#include "ESP32Servo.h"
#include "esp_adc_cal.h"
#include "movingAvg.h"    // included J. Christensen's library github: https://github.com/JChristensen/movingAvg

Servo myServo;  // Servo that controls thumb
movingAvg avgPot1(10); // create moving average object 
movingAvg avgPot2(10); 

#define AN_Pot1    34   // Thumb pot pin for position feedback
#define AN_Pot2    35   // Thumb pot pin for position feedback
#define FILTER_LEN 20   // length of 20 points, longer it is, smoother filter but takes longer time

//uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0}; // empty buffer
//int AN_Pot1_i = 0;                         // buffer index

int potVal1, potVal2; // variable to read the value from the analog pin
int Pot1_Result = 0; int Pot2_Result = 0;
float Pot1Final = 0.0; 
float Pot2Final = 0.0;
float pot1Voltage, pot2Voltage, angle1, anglesec, angle2;   // variable to hold the angle1 for the servo motor (500-2500)usec

void setup() {
  Serial.begin(115200); // open a serial connection to your computer
  delay(1000);          // allow some time to set up serial monitor
  
  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);
  // According to results provided by a problem, there's an offset of 0.1V and saturation at 1.85V. 
  // Link for the test results: https://esp32.com/viewtopic.php?f=19&t=2881&p=13739#p13739
  
  // attaches the servo on pin 14 to the servo object
  myServo.attach(14); // attaches the servo on pin 14 to the servo object
  avgPot1.begin();    // initializes the object
  avgPot2.begin();  
}

void loop() {
  potVal1 = analogRead(AN_Pot1); // read the value of the potentiometer
  int avg1 = avgPot1.reading(potVal1);
  pot1Voltage = readADC_Cal(avg1);     // calibrate it.
  Pot1Final = Voltage_Cal(pot1Voltage);  // Trendline equation from data collected comparing measurements from DMM (V) Vs ESP32 (V)
  
  potVal2 = analogRead(AN_Pot2); // read the value of the potentiometer
  //Pot2_Result = readADC2_Avg(potVal2); // go through filtering
  int avg2 = avgPot2.reading(potVal2);
  pot2Voltage = readADC_Cal(avg2);     // calibrate it.
  //pot2Voltage = Pot2_Result*3.285/4095; // convert to voltage
  Pot2Final = Voltage_Cal(pot2Voltage); 

  // scale the numbers from the pot
  angle1 = map(Pot1Final, 0.96, 3.07, 0, 300); // the values 0.96V and 3.07V came from linear range measured of ADC 
  angle2 = map(Pot2Final, 0.96, 3.07, 300, 0); 
  // map degrees into duty cycles (in microsec)
  anglesec = map(Pot1Final, 0.96, 3.07, 500, 2500);
  
  Serial.print(potVal1); 
  // print out the value to the Serial Monitor
  Serial.print(", Pot1Voltage: "); Serial.print(Pot1Final);
  // print out the angle set for the servo motor
  Serial.print(", angle1: "); Serial.print(angle1); Serial.print(", ");
  Serial.print(potVal2);
  // print out the value to the Serial Monitor
  Serial.print(", Pot2Voltage: "); Serial.print(Pot2Final);
  
  // print out the measured angle of the servo motor
  Serial.print(", angle2: "); Serial.println(angle2);

  // set the servo position
  myServo.writeMicroseconds(anglesec);
  // wait for the servo to get there
  delay(150); 

}

uint32_t readADC_Cal(int ADC_Raw)  // ADC calibration (returns in mV)
{
  esp_adc_cal_characteristics_t adc_chars;

  // ADC1, 11dB (0-4095), 12 bits, default Vref = 1.1V
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars); // address of adc_chars
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}
/*
uint32_t readADC_Avg(uint32_t arr[], int ADC_Raw) //moving average filter
{
  int i = 0;
  uint32_t Sum = 0;
  uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0}; // empty buffer
  int AN_Pot1_i = 0; 
  
  AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
  if(AN_Pot1_i == FILTER_LEN) // if buffer index full, then reset index
  {
    AN_Pot1_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {
    Sum += AN_Pot1_Buffer[i];
    arr[i] = AN_Pot1_Buffer[i];
  }
  return (Sum/FILTER_LEN);
}*/

float Voltage_Cal(float FilteredVolt) // takes float type input (in mV), if use my own conversion, then (V)
{
  float FinalVolt = 0;
  return FinalVolt = 0.9955*FilteredVolt/1000 - 0.0535; //trendline equation obtained from data collected comparing measurements from DMM (V) Vs ESP32 (V) 
}
