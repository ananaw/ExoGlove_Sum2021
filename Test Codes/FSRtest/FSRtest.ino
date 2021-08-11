/* Testing of the FSR Interlink 400 sensor 
 *  
 *  July 1, 2021
 *  by Ananda Aw
 *  San Francisco State University
 *  Insert link into preferences everytime: https://dl.espressif.com/dl/package_esp32_index.json
 */
#define fsr1pin 34
#define fsr1buffered 35
#define Vref 3.3
#define kernelsize 21
#define samplesize 5
//#define Rg 1000
#define filter_len 21   // kaiser filter length

int outputsize = kernelsize + samplesize - 1; // convolution output array size
int inputArr[21];
int i = 1;

float voltage; 
volatile unsigned long fsr1Val;  // variable to read the value from the analog pin
volatile unsigned long fsr1bufferedval;

int filtered1, filtered2;
// filter coefficients, obtained from matlab kaiser LP filter.
float tempenc1_1[filter_len], tempenc1_2[filter_len], tempenc2_1[filter_len], tempenc2_2[filter_len];
float kernelcoef[] = {0.0146, 0.0217,  0.0293, 0.0371, 0.0450, 0.0524, 0.0590, 0.0646, 0.0688, 0.0714, 0.0723,
                        0.0714, 0.0688,  0.0646, 0.0590, 0.0524, 0.0450, 0.0371, 0.0293, 0.0217, 0.0146};

void setup() {
  Serial.begin(115200); // open a serial connection to your computer
   // Setup ADC resolution and attenuation
  analogReadResolution(12); // set resolution to 11 bits (0-2047)
  //analogSetPinAttenuation(fsr1Pin, ADC_6db); // sets 6dB attenuation for 4,13,14,16 pins: 6dB range = 0-2.2V
 
}

void loop() {
  // put your main code here, to run repeatedly:
  fsr1Val = analogRead(fsr1pin);
  fsr1bufferedval = analogRead(fsr1buffered); 

  filtered1 = conv_rt(fsr1Val, tempenc1_1, tempenc1_2);
  filtered2 = conv_rt(fsr1bufferedval, tempenc2_1, tempenc2_2);

  voltage = filtered1*3.45/4095; // 3.45 because that was what was measured to be max from DMM
  float volt2 = filtered2*3.45/4095;
  
  //Serial.print(fsr1Val); Serial.print(",");
  //Serial.print(fsr1bufferedval); Serial.print(",");
  //Serial.print(filtered1); Serial.print(",");
  //Serial.print(filtered2); Serial.print(",");
  Serial.print(voltage); Serial.println(",");
  //Serial.print(volt2); Serial.print(","); Serial.println();
  //Serial.print(", Vout: "); 
  //Serial.println(Vout); //Serial.print(","); 

  delay(100);

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
/*
int conv_rt(int input[], int kernel[]) // convolution function
{
  int output[outputsize];
  int i,j;

  for (i=0; i<outputsize; i++) // size of output
  {
    output[i] = 0;
    
    for (j=0; j<kernelsize;j++) // filter size
    {
      if((i-j) < 0) // if index negative, make element 0.
      {
        input[i-j] = 0;
      }

      output[i] += input[i-j]*kernel[j]; // convolve: multiply and sum
    }
  }
  return output;
}*/
