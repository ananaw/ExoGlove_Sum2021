// Encoder Testing Code
// Testing with filtering
// by Ananda Aw, with help from Rodrigo Aceves
#define ENC1 32
#define unfiltered 33

int angle1, raw; // variable to store analog value
float voltage;

#define filter_len 21   // kaiser filter length
float filtered;
float temp[filter_len];
float temp2[filter_len];
float kernelcoef[] = {0.0146, 0.0217,  0.0293, 0.0371, 0.0450, 0.0524, 0.0590, 0.0646, 0.0688, 0.0714, 0.0723,
                        0.0714, 0.0688, 0.0646, 0.0590, 0.0524, 0.0450, 0.0371, 0.0293, 0.0217, 0.0146};
//float kernelcoef[] = {0.0323, 0.0682, 0.1060, 0.1377, 0.1557, 0.1557, 0.1377, 0.1060, 0.0682, 0.0323};

// moving average
#define FILTER_LEN 20 
uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0}; // empty buffer
int AN_Pot1_i = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);

  analogReadResolution(12); // 12 bits 0-4095
}

void loop() {
  // put your main code here, to run repeatedly:
  angle1 = analogRead(ENC1);
  raw = analogRead(unfiltered);
  filtered = conv_rt(raw);  //filtered = readADC_Avg(angle1, AN_Pot1_Buffer);
  voltage = angle1*3.3/4095; // convert to voltage
  //float voltfiltered = filtered*3.3/4095;
  float anglemap = map(angle1, 0, 4095, 0, 333);
  float anglefilt = map(filtered, 0, 4095, 0, 333);\
  float angleraw = map(raw, 0, 4095, 0, 333);
  //int angle = round(anglefilt);
  //int angle2 = round(angleraw);
  
  Serial.print(angleraw); Serial.print(","); 
  //Serial.print(", angle filtered: "); 
  Serial.print(anglefilt); Serial.print(","); Serial.println();
  delay(100);
}

float conv_rt(int input)
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

uint32_t readADC_Avg(int ADC_Raw, uint32_t buffer1[]) //moving average filtering
{
  int i = 0;
  uint32_t Sum = 0;
  
  buffer1[AN_Pot1_i++] = ADC_Raw;
  if(AN_Pot1_i == FILTER_LEN) // if buffer index full, then reset index
  {
    AN_Pot1_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {
    Sum += buffer1[i];
  }
  return (Sum/FILTER_LEN);
}
