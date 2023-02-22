#include <Servo.h>
int potPin = A0;
int potVal = 0;
int redLed = 9;
Servo servo;
int servoPin = 10;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //pinMode(redLed,OUTPUT);
  servo.attach(servoPin);
}

void loop() {
  potVal = analogRead(potPin);
  potVal=map(potVal, 0, 1023, 0, 180);
  analogWrite(redLed,potVal);
  servo.write(potVal);
  Serial.println(potVal);
  delay(15);
  // put your main code here, to run repeatedly:

}
