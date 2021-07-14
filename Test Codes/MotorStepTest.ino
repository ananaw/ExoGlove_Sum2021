#include "ESP32Servo.h"

Servo myServo;

#define AN_Pot1 35

int potVal1;
int potAngle;
int dutycycle = 500;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myServo.attach(14);
  myServo.writeMicroseconds(500); //reset servo
}

void loop() {
  
  // put your main code here, to run repeatedly:
//  for(int i = 500; i<2501; i+=10)
//  {
//    myServo.writeMicroseconds(i);
//    potVal1 = analogRead(AN_Pot1);
//    potAngle = map(potVal1, 0, 4095, 0, 270); // saturation of 40.6 degrees in beginning and 25.6 degrees near end of knob. 
    //}
    if(dutycycle > 2500)
    {
      dutycycle = 500;
    }
    
    myServo.writeMicroseconds(dutycycle); // step servo, servo actually moves to 260 deg only
    delay(500); // delay 0.5 sec (motor speed at 7.4V is 0.11sec/60deg)
    
    potVal1 = analogRead(AN_Pot1);
    potAngle = map(potVal1, 0, 4095, 41, 270); // saturation of 40.6 degrees in beginning and 25.6 degrees near end of knob. 
    Serial.print("Analog: "); Serial.print(potVal1);
    Serial.print(", Angle: "); Serial.println(potAngle);
    dutycycle += 100;
    delay(500);
    
    
   
}
