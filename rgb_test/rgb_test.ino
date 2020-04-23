int red_light_pin= 11;
int green_light_pin = 10;
int blue_light_pin = 9;

int brightness = 50; // main brightness

void setup() {
  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);
}
void loop() {

  /*RGB_color(brightness, 0, 0); // Red
  delay(1000);  
  RGB_color(100, 10, 0); // Orange
  delay(1000);*/
  
  RGB_color(0, brightness, brightness); // Cyan
  
  /*delay(1000);
  RGB_color(0, brightness, 2); // Green
  delay(1000);*/
  
  /*
  RGB_color(0, 0, brightness); // Blue
  delay(1000);
  RGB_color(brightness, brightness, 125); // Raspberry
  delay(1000);
  zRGB_color(brightness, 0, brightness); // Magenta
  delay(1000);
  RGB_color(brightness, brightness, 0); // Yellow
  delay(1000);
  */

  

}
void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
 {
  analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}
