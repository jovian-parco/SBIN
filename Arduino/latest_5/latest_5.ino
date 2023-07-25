#include <Servo.h>
#include <NoDelay.h>
#include <NewPing.h>
#include <TimerFreeTone.h>

String biodegradable[] = 
{
  "bio",
  "paper_carton", 
  "paper_crumpled", 
  "paper_cup", 
  "paper_container",
  "paper_sheet", 
  "paper_tissue"
};
String non_biodegradable[] = 
{
  "non_bio",
  "pen", 
  "plastic_bag", 
  "plastic_bottle",  
  "plastic_cup", 
  "plastic_lid", 
  "plastic_straw", 
  "plastic_utensils",  
  "candy_wrapper"
};
String reject[] = 
{
  "no object detected",
  "reject",
  "facemask"
};

#define wiper_servo_pin 2
#define pltf_servo_pin 3
#define buzzer_pin 4
#define cam_t_us_pin 5
#define cam_e_us_pin 6
#define ltrash_t_us_pin 9
#define ltrash_e_us_pin 10
#define rtrash_t_us_pin 7
#define rtrash_e_us_pin 8
#define red_led_pin 11
#define yellow_led_pin 12
#define green_led_pin 13
#define water_level_pin A0
#define water_pump_pin A1
#define us_max_distance 400

unsigned long buzzer_frequeny = 3000;
const int water_level_threshold = 370; //370
const int reject_wiper_min_pos = 0;
const int reject_wiper_max_pos = 180;
const int left_pltf_max_pos = 60;
const int right_pltf_max_pos = 130;
const int reject_pltf_max_pos = 105;
const int center_pltf_pos = 88;
const int pltf_delay = 2000;
const int cam_us_max_distance = 31;
const int ltrash_us_max_distance = 30;
const int rtrash_us_max_distance = 30;
noDelay us_cam_detect_interval(10);
noDelay water_pump_interval(900000); //15mins 
const int water_pump_on_interval(5000);
noDelay water_trash_detect_interval(1000);

const int servo_delay = 15;
int wiper_servo_pos = 0;
int pltf_servo_pos = 0;
bool busy_camera = false;
bool busy_water_trash = false;
bool buzzer_state = false;
bool rpi_state = false;
noDelay half_sec(500);
noDelay one_sec(1000);
noDelay two_sec(2000);
noDelay three_sec(3000);

NewPing rtrash_us(rtrash_t_us_pin, rtrash_e_us_pin, us_max_distance);
NewPing ltrash_us(ltrash_t_us_pin, ltrash_e_us_pin, us_max_distance);
NewPing cam_us(cam_t_us_pin, cam_e_us_pin, us_max_distance);
Servo wiper_servo;
Servo pltf_servo;
String command;
int biodegradable_size = sizeof(biodegradable) / sizeof(biodegradable[0]);
int non_biodegradable_size = sizeof(non_biodegradable) / sizeof(non_biodegradable[0]);
int reject_size = sizeof(reject) / sizeof(reject[0]);

void loop() {
  led_activate();
  buzzer_activate();
  
  if (us_cam_detect_interval.update() && cam_us.convert_cm(cam_us.ping_median(3)) != cam_us_max_distance && busy_camera == false && busy_water_trash == false && rpi_state == true)
  {
    busy_camera = true;  
    detected();
  }  

  if(water_trash_detect_interval.update());
  {
    int water_level_value = analogRead(water_level_pin);
    if(rtrash_us.ping_cm() <= rtrash_us_max_distance && busy_camera == false)
    {
      busy_water_trash = true;
      Serial.println("rtrash_full");
      buzzer_generate(.5);
    }
    else if(ltrash_us.ping_cm() <= ltrash_us_max_distance && busy_camera == false)
    {
      busy_water_trash = true;
      Serial.println("ltrash_full");
      buzzer_generate(.5);
    }
    else if (water_level_value < water_level_threshold && busy_camera == false) 
    {
      busy_water_trash = true;
      Serial.println("water_empty");
      buzzer_generate(3);
      Serial.println(water_level_value);
    }
    else
    {
    busy_water_trash = false;
    buzzer_generate(0);
    }
  }

  if(water_pump_interval.update() && busy_water_trash == false && busy_camera == false)
  {
    Serial.println("water_pump_on");
    water_pump_on();
    delay(water_pump_on_interval);
    water_pump_off();
    Serial.println("water_pump_off");
  }

  if (Serial.available())
  {
    command = Serial.readStringUntil("\n");
    command.trim();
    if(command.equals("rpi ready"))
    {
      rpi_state = true;
    }
    for (int i = 0; i < biodegradable_size; i++)
    {
      if (command.equals(biodegradable[i]))
      {    
        led_busy();
        pltf_right();
        busy_camera = false;
        led_available();
        return;
      }
    } 
    for (int i = 0; i < non_biodegradable_size; i++)
    {
      if (command.equals(non_biodegradable[i]))
      {
        led_busy();
        pltf_left();
        busy_camera = false;
        led_available();
        return;
      }
    }
    for (int i = 0; i < reject_size; i++)
    {
      if (command.equals(reject[i]))
      {
        led_busy();
        pltf_reject();
        busy_camera = false;
        led_available();
        return;
      }
    } 
  }

}

void led_activate()
{
  if (busy_camera == true)
  {
    led_processing();
  }
  else if(busy_camera == false && busy_water_trash == false && rpi_state == true)
  {
    led_available();
  }   
  else if(busy_water_trash == true || rpi_state == false)
    led_busy();  
}

void buzzer_generate(float sec)
{
  if(sec == 0)
  { 
    buzzer_state = false;
  }
  if(sec == .5 && half_sec.update())
  { 
    buzzer_state = !buzzer_state;
  }
  if(sec == 1 && one_sec.update())
  { 
    buzzer_state = !buzzer_state;
  }
  if(sec == 2 && two_sec.update())
  { 
    buzzer_state = !buzzer_state;
  }
  if(sec == 3 && three_sec.update())
  { 
    buzzer_state = !buzzer_state;
  }
}

void buzzer_activate()
{
  if (buzzer_state)
  {
    TimerFreeTone(buzzer_pin,buzzer_frequeny,10,100);
  }
  else 
  {
    TimerFreeTone(buzzer_pin,buzzer_frequeny,0,1);
  }
}

void water_pump_on()
{
  digitalWrite(water_pump_pin, LOW);
}

void water_pump_off()
{
  digitalWrite(water_pump_pin, HIGH);
}

void led_busy()
{
  green_led_off();
  yellow_led_off();
  red_led_on();
}

void led_processing()
{
  green_led_off();
  yellow_led_on();
  red_led_off();
}

void led_available()
{
  green_led_on();
  yellow_led_off();
  red_led_off();
}


void red_led_on()
{
  digitalWrite(red_led_pin, HIGH);
}

void yellow_led_on()
{
  digitalWrite(yellow_led_pin, HIGH);
}

void green_led_on()
{
  digitalWrite(green_led_pin, HIGH);
}

void red_led_off()
{
  digitalWrite(red_led_pin, LOW);
}

void yellow_led_off()
{
  digitalWrite(yellow_led_pin, LOW);
}

void green_led_off()
{
  digitalWrite(green_led_pin, LOW);
}

void pltf_reject()
{
  for (pltf_servo_pos = center_pltf_pos; pltf_servo_pos <=reject_pltf_max_pos; pltf_servo_pos +=1)
  {
    pltf_servo.write(pltf_servo_pos);
    delay(servo_delay);
  }

  for (wiper_servo_pos = reject_wiper_min_pos; wiper_servo_pos <= reject_wiper_max_pos; wiper_servo_pos +=1)
  {
    wiper_servo.write(wiper_servo_pos);
    delay(servo_delay);
  }

  for (wiper_servo_pos = reject_wiper_max_pos; wiper_servo_pos >= reject_wiper_min_pos; wiper_servo_pos -=1)
  {
    wiper_servo.write(wiper_servo_pos);
    delay(servo_delay);
  } 

  for (pltf_servo_pos = reject_pltf_max_pos; pltf_servo_pos >=center_pltf_pos; pltf_servo_pos -=1)
  {
    pltf_servo.write(pltf_servo_pos);
    delay(servo_delay);
  }  
  Serial.println("pltf_reject_done");
}

void  pltf_left()
{
  for (pltf_servo_pos = center_pltf_pos; pltf_servo_pos >=left_pltf_max_pos; pltf_servo_pos -=1)
  {
    pltf_servo.write(pltf_servo_pos);
    delay(servo_delay);
  }
  delay(pltf_delay);
  for (pltf_servo_pos = left_pltf_max_pos; pltf_servo_pos <=center_pltf_pos; pltf_servo_pos +=1)
  {
    pltf_servo.write(pltf_servo_pos);
    delay(servo_delay);
  }
  Serial.println("pltf_left_done");
}

void  pltf_right()
{
  for (pltf_servo_pos = center_pltf_pos; pltf_servo_pos <=right_pltf_max_pos; pltf_servo_pos +=1)
  {
    pltf_servo.write(pltf_servo_pos);
    delay(servo_delay);
  }
  delay(pltf_delay);
  for (pltf_servo_pos = right_pltf_max_pos; pltf_servo_pos >=center_pltf_pos; pltf_servo_pos -=1)
  {
    pltf_servo.write(pltf_servo_pos);
    delay(servo_delay);
  }  
  Serial.println("pltf_right_done");
}

void detected()
{
  Serial.println("cam_detected");
}

void setup() 
{
  Serial.begin(9600);
  pinMode(buzzer_pin, OUTPUT);
  pinMode(red_led_pin,OUTPUT);
  pinMode(yellow_led_pin,OUTPUT);
  pinMode(green_led_pin,OUTPUT);
  pinMode(water_pump_pin,OUTPUT);
  wiper_servo.attach(wiper_servo_pin);
  pltf_servo.attach(pltf_servo_pin);
  red_led_on();
  yellow_led_on();
  green_led_on();
  yellow_led_off();
  green_led_off();
  water_pump_off();
  pltf_reject();
}

