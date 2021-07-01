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

Servo myServo;  // create a servo object

int const potPin1 = 13; // analog pin used to connect the potentiometer
int const potPin2 = 21; // analog pin used to connect the potentiometer
int potVal1;  // variable to read the value from the analog pin
int potVal2;  // variable to read the value from the analog pin
int pot1Voltage, pot2Voltage, angle1, anglesec, angle2;   // variable to hold the angle1for the servo motor (500-2500)usec

void setup() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3); 
  
  // Setup ADC
  analogReadResolution(11); // set resolution to 11 bits (0-2047)
  analogSetAttenuation(13||21, ADC_6db); // sets 6dB attenuation for 4,13,14,16 pins: 6dB range = 0-2.2V
  // According to results provided by a problem, there's an offset of 0.1V and saturation at 1.85V. 
  // Link for the test results: https://esp32.com/viewtopic.php?f=19&t=2881&p=13739#p13739
  
  // attaches the servo on pn 25 to the servo object, set min & max us
  myServo.attach(25); // attaches the servo on pin 25 to the servo object
  Serial.begin(115200); // open a serial connection to your computer
}

void loop() {
  //an  gle1= 500;
  potVal1 = analogRead(potPin1); // read the value of the potentiometer
  potVal2 = analogRead(potPin2); // read the value of the potentiometer
  pot1Voltage = 3.3*potVal1/2047; // convert into voltage;
  pot2Voltage = 3.3*potVal2/2047;
  
  // scale the numbers from the pot
  angle1= map(potVal1, 95, 1145, 0, 300); // the values 93 and 1148 came from converting linear range (0.15-1.85V) of ADC 
  // scale the numbers from the pot
  angle2= map(potVal2, 95, 1145, 300, 0);
  // map degrees into duty cycles (in microsec)
  anglesec = map(potVal1, 95, 1145, 500, 2500);
  
  // print out the value to the Serial Monitor
  Serial.print("potVal1: "); Serial.print(potVal1);
  // print out the value to the Serial Monitor
  Serial.print(", potVal2: "); Serial.print(potVal2);
  // print out the angle set for the servo motor
  Serial.print(", angle1: "); Serial.print(angle1);
  // print out the measured angle of the servo motor
  Serial.print(", angle2: "); Serial.print(angle2);
   // print out the angle set for the servo motor
  Serial.print(", Voltage1: "); Serial.print(pot1Voltage);
   // print out the angle set for the servo motor
  Serial.print(", Voltage2: "); Serial.println(pot2Voltage);

  // set the servo position
  myServo.writeMicroseconds(anglesec);
  // wait for the servo to get there
  delay(150); 

}
