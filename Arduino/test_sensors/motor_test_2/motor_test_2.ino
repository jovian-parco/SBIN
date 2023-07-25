#include <Servo.h>

Servo main_servo;

int main_servo_pos = 0;

void setup() {
  main_servo.attach(3);

}

void loop() {
  main_servo.write(90);
  delay(7000);
  for (main_servo_pos = 90; main_servo_pos >=0; main_servo_pos -=1)
  {
    main_servo.write(main_servo_pos);
    delay(15);
  }
  for (main_servo_pos = 00; main_servo_pos <=90; main_servo_pos +=1)
  {
    main_servo.write(main_servo_pos);
    delay(15);
  }
  for (main_servo_pos = 90; main_servo_pos <=180; main_servo_pos +=1)
  {
    main_servo.write(main_servo_pos);
    delay(15);
  }
  for (main_servo_pos = 180; main_servo_pos >=90; main_servo_pos -=1)
  {
    main_servo.write(main_servo_pos);
    delay(15);
  }  

}