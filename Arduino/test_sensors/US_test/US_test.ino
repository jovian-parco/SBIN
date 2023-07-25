#include <Servo.h>
#include <NoDelay.h>
#include <NewPing.h>


#define cam_t_us_pin 5
#define cam_e_us_pin 6
#define ltrash_t_us_pin 7
#define ltrash_e_us_pin 8
#define rtrash_t_us_pin 9
#define rtrash_e_us_pin 10


#define us_max_distance 400
bool busy = false;

//32 trash max
const int cam_us_max_distance = 20;
const int ltrash_us_max_distance = 30;

noDelay us_cam_delay(10);
noDelay us_ltrash_delay(5000);
NewPing cam_us(cam_t_us_pin, cam_e_us_pin, us_max_distance);
NewPing ltrash_us(ltrash_t_us_pin, ltrash_e_us_pin, us_max_distance);
int news;
int data;
void setup() {
  Serial.begin(9600);

}

void loop() {

  
    if (us_cam_delay.update())
  {
    Serial.print("cam");
    data = cam_us.convert_cm(cam_us.ping_median(3));
    //news = map(data, 375, 1743,0 ,100);
    Serial.println(data);
  }
}
