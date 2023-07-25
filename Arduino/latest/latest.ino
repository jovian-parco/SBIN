#include <Servo.h>

String command;
Servo servo;

#define greenled 8
#define yellowled 9
#define redled 10
#define buttonpin 11
#define servopin 12



int oldbuttonstate = 1;
int button;
bool printenable = true;
bool serialenable = true;
bool busy = false;

void redled_on()
{
  digitalWrite(redled, HIGH);
}

void yellowled_on()
{
  digitalWrite(yellowled, HIGH);
}

void greenled_on()
{
  digitalWrite(greenled, HIGH);
}

void redled_off()
{
  digitalWrite(redled, LOW);
}

void yellowled_off()
{
  digitalWrite(yellowled, LOW);
}

void greenled_off()
{
  digitalWrite(greenled, LOW);
}

void turn_right()
{
  servo.write(0);
  Serial.println("right motor");
  greenled_off();
  yellowled_off();
  redled_on(); 
}

void turn_left(){
  servo.write(180);
  Serial.println("left motor");
  greenled_off();
  yellowled_off();
  redled_on();
}

void turn_center(){
  servo.write(90);
  Serial.println("center motor");
  greenled_on();
  yellowled_off();
  redled_off();
  busy = false;
}

void setup()
{     
  Serial.begin(9600);
  pinMode(buttonpin,INPUT);
  servo.attach(servopin);
  pinMode(redled,OUTPUT);  
  pinMode(yellowled,OUTPUT);  
  pinMode(greenled,OUTPUT);    
}

void loop() 
{
  button = digitalRead(buttonpin);
  
  if (busy == true)
  {
    greenled_off();
    yellowled_on();
    redled_off();
  }
  else
  {
    greenled_on();
    yellowled_off();
    redled_off();
  }   
  
  if ((button == 1)&&(busy == false))  
  {
    busy = true;
    Serial.println("pressed button");      
  }

  if (Serial.available())
  {
    command = Serial.readStringUntil("\n");
    command.trim();

    if (command.equals("right"))
    {    
      turn_right();
      delay(1000);
      turn_center();
    }
    else if (command.equals("left"))
    {
      turn_left();
      delay(1000);
      turn_center();      
    }
  }  



}
