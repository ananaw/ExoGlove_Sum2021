/* Testing of the FSR Interlink 400 sensor 
 *  
 *  July 1, 2021
 *  by Ananda Aw
 *  San Francisco State University
 *  Insert link into preferences everytime: https://dl.espressif.com/dl/package_esp32_index.json
 */
#define Vref 5.0;
int const fsr1Pin = 19; 

volatile unsigned long fsr1Val;  // variable to read the value from the analog pin

void setup() {
  // put your setup code here, to run once:
   // Setup ADC resolution and attenuation
  analogReadResolution(11); // set resolution to 11 bits (0-2047)
  analogSetPinAttenuation(fsr1Pin, ADC_6db); // sets 6dB attenuation for 4,13,14,16 pins: 6dB range = 0-2.2V
  
  Serial.begin(115200); // open a serial connection to your computer
}

void loop() {
  // put your main code here, to run repeatedly:
  fsr1Val = analogRead(fsr1Pin);
  // Vout = (Vref/2)*(1-Rg/Rfsr);
  Serial.print("analog: "); Serial.println(fsr1Val);

  delay(10);
}
