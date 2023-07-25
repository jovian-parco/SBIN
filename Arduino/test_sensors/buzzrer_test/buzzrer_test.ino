#include <NewPing.h>
#include <TimerFreeTone.h>
#include <NoDelay.h>
#include <Servo.h>
#define buzzer_pin 9

bool buzzer_state = false;
noDelay halfsec(500);
noDelay onesec(1000);
noDelay twosec(2000);
noDelay threesec(3000);


void setup() 
{
  // put your setup code here, to run once:
  pinMode(buzzer_pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  sound(1);
  state();

}

void sound(float sec)
{ 
  if(sec == 1 && onesec.update())
  { 
    buzzer_state = !buzzer_state;
  }
  if(sec == .5 && halfsec.update())
  { 
    buzzer_state = !buzzer_state;
  }
  
}
void state()
{
  if (buzzer_state)
  {
    TimerFreeTone(buzzer_pin,500,5,10);
  }
  else 
  {
    TimerFreeTone(buzzer_pin,500,0,1);
  }
} 



  

