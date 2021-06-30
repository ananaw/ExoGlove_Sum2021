/*
Testing of the 2000 series dual mode motor with cable, possibly with potentiometer

  Parts required:
  - servo motor
  - 10 kilohm potentiometer

June 29, 2021
by Ananda Aw
*/

// include the Servo library
//#include <Servo.h>
#include "ESP32Servo.h"

Servo myServo;  // create a servo object

int const potPin1 = 13; // analog pin used to connect the potentiometer
int const potPin2 = 14; // analog pin used to connect the potentiometer
int potVal1;  // variable to read the value from the analog pin
int potVal2;  // variable to read the value from the analog pin
int angle1, anglesec, angle2;   // variable to hold the angle1for the servo motor (500-2500)usec

void setup() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  // attaches the servo on pin 25 to the servo object, set min & max us
  myServo.attach(25); // attaches the servo on pin 25 to the servo object
  Serial.begin(115200); // open a serial connection to your computer
}

void loop() {
  //an  gle1= 500;
  analogReadResolution(12); // set resolution to 12 bits (0-4095)
  potVal1 = analogRead(potPin1); // read the value of the potentiometer
  potVal2 = analogRead(potPin2); // read the value of the potentiometer
  
  // scale the numbers from the pot
  angle1= map(potVal1, 0, 4095, 0, 300); 
  // scale the numbers from the pot
  angle2= map(potVal2, 0, 4095, 300, 0);
  // map degrees into duty cycles (in microsec)
  anglesec = map(potVal1, 0, 4095, 500, 2500);
  
  // print out the value to the Serial Monitor
  Serial.print("potVal1: "); Serial.print(potVal1);
  // print out the value to the Serial Monitor
  Serial.print(", potVal2: "); Serial.print(potVal2);
  // print out the angle set for the servo motor
  Serial.print(", angle1: "); Serial.print(angle1);
  // print out the measured angle of the servo motor
  Serial.print(", angle2: "); Serial.println(angle2);

  // set the servo position
  myServo.writeMicroseconds(anglesec);
  // wait for the servo to get there
  delay(150); 

}
