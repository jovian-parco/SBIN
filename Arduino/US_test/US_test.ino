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

noDelay us_cam_delay(50);
noDelay us_ltrash_delay(5000);
NewPing cam_us(cam_t_us_pin, cam_e_us_pin, us_max_distance);
NewPing ltrash_us(ltrash_t_us_pin, ltrash_e_us_pin, us_max_distance);

void setup() {
  Serial.begin(9600);

}

void loop() {
  // if (us_cam_delay.update() && cam_us.convert_cm(cam_us.ping_median(5)) <= cam_us_max_distance)
  // {
  //   Serial.print("cam");
  //   Serial.println(cam_us.convert_cm(cam_us.ping_median(5)));
  // }
  // if (us_ltrash_delay.update() && ltrash_us.convert_cm(ltrash_us.ping_median(5)) <= ltrash_us_max_distance && busy == false)
  // {
  //   busy = true;
  //   Serial.print("ltrash_us puno");
  //   Serial.println(ltrash_us.convert_cm(ltrash_us.ping_median(5)));
  // }
  // if (us_ltrash_delay.update() && ltrash_us.convert_cm(ltrash_us.ping_median(5)) >= ltrash_us_max_distance && busy == true)
  // {
  //   busy = false;
  //   Serial.print("ltrash_us dipuno");
  //   Serial.println(ltrash_us.convert_cm(ltrash_us.ping_median(5)));
  // }
  // if (us_ltrash_delay.update())
  // {
  //   if (ltrash_us.convert_cm(ltrash_us.ping_median(5)) <= ltrash_us_max_distance)
  //   {
  //     busy = true;
  //     Serial.print("ltrash_us puno");
  //     Serial.println(ltrash_us.convert_cm(ltrash_us.ping_median(5)));
  //   }
  //   else {
  //     busy = false;
  //     Serial.print("ltrash_us dipuno");
  //     Serial.println(ltrash_us.convert_cm(ltrash_us.ping_median(5)));
  //   }
  // }
    if (us_cam_delay.update())
  {
    Serial.print("cam");
    Serial.println(cam_us.convert_cm(cam_us.ping_median(10)));
  }
}
