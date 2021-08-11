/*
 * Name: ESP32 ADC Calibration
 * Author: Ananda Aw, referenced from DeepBlueMbedded.com 
 * For More Info Visit: www.DeepBlueMbedded.com
 * with additional calibration, it's accurate up to about +-0.01 V 
*/
#include "esp_adc_cal.h"
#define AN_Pot1    13
#define FILTER_LEN 20 

uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0}; // empty buffer
int AN_Pot1_i = 0;                         // buffer index
int AN_Pot1_Raw = 0;
int AN_Pot1_Result = 0;
float Pot1Final = 0.0;
float Voltage = 0.0;
void setup() 
{
  Serial.begin(115200);
}
void loop() 
{
  AN_Pot1_Raw = analogRead(AN_Pot1);
  AN_Pot1_Result = readADC_Avg(AN_Pot1_Raw); // go through filtering
  Voltage = readADC_Cal(AN_Pot1_Result);     // calibrate it.
  Pot1Final = 0.9955*Voltage/1000 - 0.0535;  // Trendline equation from data collected comparing measurements from DMM (V) Vs ESP32 (V)
  Serial.print("Filtered: ");              // filtered result
  Serial.print(AN_Pot1_Result);              // filtered result
  Serial.print(", ");
  Serial.print(Voltage/1000, 3); // Print Voltage (in V) reads to 3.277 V on DMM, 3.14V on esp32
  Serial.print(", Final: ");
  Serial.println(Pot1Final, 3); // Print final voltage
  //Serial.println(Voltage);      // Print Voltage (in mV)
  delay(100);
}

uint32_t readADC_Cal(int ADC_Raw)  // ADC calibration
{
  esp_adc_cal_characteristics_t adc_chars;

  // ADC1, 11dB (0-4095), 12 bits, default Vref = 1.1V
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars); // address of adc_chars
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

uint32_t readADC_Avg(int ADC_Raw) //moving average filtering
{
  int i = 0;
  uint32_t Sum = 0;
  
  AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
  if(AN_Pot1_i == FILTER_LEN) // if buffer index full, then reset index
  {
    AN_Pot1_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {
    Sum += AN_Pot1_Buffer[i];
  }
  return (Sum/FILTER_LEN);
}
