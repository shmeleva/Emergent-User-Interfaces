#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// These variables are not changing so using constant variables
const int vMotor1 = 9;
const int vMotor2 = 10;

// Targeted respiratory rate
int respiratoryRate = 6;

// milliseconds, the break between inhale and exhale
int pause = 200;

/* There is this pause after each decrease, before the next breath starts. 
   It happens probably because the vibration frequency values get so low that the motor
   cannot produce it anymore. This is why we are starting the fading from a higher value than 0.
*/
int minFrq = 50;

// sets in how big steps the frequency changes
int fadeStep = (255-minFrq)/((60/respiratoryRate + pause*0.001)/0.2);

/* The resulting respiratory rate is result of the fadeStep value and the pauses between breaths.
 FadeStep determines how fast we move from 0 value to value 255. 
 We can calculate the respiratory rate by: 
 60s / ((255 / fadeStep * a * b * c) + pauseTime * 2)
 Where (a * b * c = 0.2)
 a = 100ms, delay between steps of vibration frequency 
 b = 2, because we want to count both inhale and exhale
 c = 0.001 because we want to result in seconds instead of millisecond
*/

/* Set the delay between fresh samples */
/*#define BNO055_SAMPLERATE_DELAY_MS (100)//This is rate accelerometer takes samples, delay in vibration should do the same, not tested*/

Adafruit_BNO055 bno = Adafruit_BNO055();
const int green_light_pin_0 = 11;
const int blue_light_pin_0 = 10;
const int green_light_pin_1 = 9;
const int blue_light_pin_1 = 8;
const int green_light_pin_2 = 7;
const int blue_light_pin_2 = 6;
const int green_light_pin_3 = 5;
const int blue_light_pin_3 = 4;

int brightness = 50;

const int buttonPin = 2; 
int buttonState = 0;
bool buttonStatus = false;

float movement[] = {0,0,0};
//int counter = 0;
float filterOne = 0.2;//filters for movement Y
float filterTwo = 0.4;//filters for movement Z
float movementVal = 1;//filter for total amount for movement to be determined
boolean notDetermined = true;
String direction = "";
int directionInt = -1;
bool directionDone = true;
bool insGiven = false;
bool feedGiven = false;

//directional instuctions
int next_z = 0;
int prev_z = 1;
int next_y = 2;
int prev_y = 3;
  
int next_movement = -1;
int axis = -1;
int temp = -1;

void accelFunc(){
  if(directionDone){ //Determine instruction
  axis = rand() % 2; // pick random axis

  // z-axis
  if (axis == 0) {
    next_movement = next_z;
    // swap the directions so next coming is the opposite
    temp = prev_z;
    prev_z = next_z;
    next_z = temp;
  }
  // y-axis
  else {
    next_movement = next_y;
    // swap the directions so next coming is the opposite
    temp = prev_y;
    prev_y = next_y;
    next_y = temp;
  }
  directionDone = false;

 }
  //tell about movement instruction here
  if(directionDone == false && notDetermined == false && insGiven == false){
    if(next_movement == 0){
      analogWrite(blue_light_pin_0, brightness);
    }
    else if(next_movement == 1){
      analogWrite(blue_light_pin_1, brightness);
    }
    else if(next_movement == 2){
      analogWrite(blue_light_pin_2, brightness);
    }
    else if(next_movement == 3){
      analogWrite(blue_light_pin_3, brightness);
    }
    insGiven == true;
  }
  //get movement data
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  /* Display the floating point data */
  float movX = acc.x();
  float movY = acc.y();
  float movZ = acc.z();
  Serial.print("Acc X: ");
  Serial.print(movX);
  Serial.print(" Y: ");
  Serial.print(movY);
  Serial.print(" Z: ");
  Serial.print(movZ);
  Serial.print("\t");

  //Filters smaller movements
  if(filterOne < movX || movX < - filterOne)movement[0] = movement[0] + movX;
  if(filterOne < movY || movY < - filterOne)movement[1] = movement[1] + movY;
  if(filterTwo < movZ || movZ< - filterTwo)movement[2] = movement[2] + movZ;

  //Y = left, -Y = right, Z = up, -Z = down
  //Tells which direction biggest movement is

  if(notDetermined){
    if(movement[1] > movementVal ){
      direction = "Left";
      directionInt = 3;
      notDetermined = false;
    }
    else if(movement[1] < -movementVal) {
      direction = "Right";
      directionInt = 2;
      notDetermined = false;
    }
    else if(movement[2] > movementVal ){
      direction = "Down";
      directionInt = 1;
      notDetermined = false;
    }
    else if(movement[2] < -movementVal) {
      direction = "Up";
      directionInt = 0;
      notDetermined = false;
    }
    if(notDetermined == false && feedGiven == false){//Checks if movement is same as instructed
      Serial.print("Intended direction: ");
      Serial.print(next_movement);
      Serial.print(" Actual movement: ");
      Serial.print(directionInt);
      if(next_movement == directionInt){
        //movement is correct
        Serial.print(" movement is correct");
        if(next_movement == 0){
         analogWrite(green_light_pin_0, brightness);
         analogWrite(blue_light_pin_0, 0); // Zero blue color
        }
        else if(next_movement == 1){
          analogWrite(green_light_pin_1, brightness);
          analogWrite(blue_light_pin_1, 0); // Zero blue color
        }
        else if(next_movement == 2){
         analogWrite(green_light_pin_2, brightness);
         analogWrite(blue_light_pin_2, 0); // Zero blue color
        }
        else if(next_movement == 3){
         analogWrite(green_light_pin_3, brightness);
         analogWrite(blue_light_pin_3, 0); // Zero blue color
        }
      }
      else if((next_movement == 0 || next_movement == 1) && (directionInt == 0 || directionInt == 1)){
        //movement is semi-correct, green light
        Serial.print(" movement is semi-correct");
        if(next_movement == 0){
         analogWrite(green_light_pin_0, brightness);
         analogWrite(blue_light_pin_0, 0); // Zero blue color
        }
        else if(next_movement == 1){
          analogWrite(green_light_pin_1, brightness);
          analogWrite(blue_light_pin_1, 0); // Zero blue color
        }
        else if(next_movement == 2){
         analogWrite(green_light_pin_2, brightness);
         analogWrite(blue_light_pin_2, 0); // Zero blue color
        }
        else if(next_movement == 3){
         analogWrite(green_light_pin_3, brightness);
         analogWrite(blue_light_pin_3, 0); // Zero blue color
        }
      }
      else if((next_movement == 2 || next_movement == 3) && (directionInt == 2 || directionInt == 3)){
        //movement is semi-correct, green light
        Serial.print(" movement is semi-correct");
        if(next_movement == 0){
         analogWrite(green_light_pin_0, brightness);
         analogWrite(blue_light_pin_0, 0); // Zero blue color
        }
        else if(next_movement == 1){
          analogWrite(green_light_pin_1, brightness);
          analogWrite(blue_light_pin_1, 0); // Zero blue color
        }
        else if(next_movement == 2){
         analogWrite(green_light_pin_2, brightness);
         analogWrite(blue_light_pin_2, 0); // Zero blue color
        }
        else if(next_movement == 3){
         analogWrite(green_light_pin_3, brightness);
         analogWrite(blue_light_pin_3, 0); // Zero blue color
        }
      }
      else{
        //movement is wrong, keep blue light
        Serial.print(" movement is wrong");
      }
      feedGiven = true;
      Serial.print("\t");
    }
  }

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
}

void movementDone(){
  movement[0] = 0;
  movement[1] = 0;
  movement[2] = 0;
  direction = "";
  notDetermined = true;
  directionDone = true;
  insGiven = false;
  feedGiven = false;

    //Zero all lights
    analogWrite(green_light_pin_0, 0);
    analogWrite(blue_light_pin_0, 0); 
    analogWrite(green_light_pin_1, 0);
    analogWrite(blue_light_pin_1, 0); 
    analogWrite(green_light_pin_2, 0);
    analogWrite(blue_light_pin_2, 0); 
    analogWrite(green_light_pin_3, 0);
    analogWrite(blue_light_pin_3, 0); 
       
}
void setup(void)
{
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
  pinMode(green_light_pin_0, OUTPUT);
  pinMode(blue_light_pin_0, OUTPUT);
  pinMode(green_light_pin_1, OUTPUT);
  pinMode(blue_light_pin_1, OUTPUT);
  pinMode(green_light_pin_2, OUTPUT);
  pinMode(blue_light_pin_2, OUTPUT);
  pinMode(green_light_pin_3, OUTPUT);
  pinMode(blue_light_pin_3, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Accel Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

void loop() {

  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  if(buttonState == HIGH){
    if(buttonStatus == true)buttonStatus = false;
    else buttonStatus = true;
    delay(1000);
  }
  if(buttonStatus){
    
  
  // fade in from min to max in increments of 5 points:
  for (int fadeValue = minFrq; fadeValue <= 255; fadeValue += fadeStep) {
    
    // sets the value (range from 0 to 255):
    analogWrite(vMotor1, fadeValue);
    analogWrite(vMotor2, fadeValue);


    accelFunc();

    buttonState = digitalRead(buttonPin);
    if(buttonState == HIGH){//Checks if button is pressed
      if(buttonStatus == true){
        buttonStatus = false;
        break;
      }
    }
    
    // sets how long delay is betweeen the frequency changes 
    // ! Changing this will affect the respiratory rate
    delay(100);
  }

  movementDone();

  }

  if(buttonStatus){
    
  
  // Stop for a while during the peak before exhale instructions start
  analogWrite(vMotor1, 0);
  analogWrite(vMotor2, 0);

  delay(pause);

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255; fadeValue >= minFrq; fadeValue -= fadeStep) {
    
    // sets the value (range from 0 to 255):
    analogWrite(vMotor1, fadeValue);
    analogWrite(vMotor2, fadeValue);

    accelFunc();

    buttonState = digitalRead(buttonPin);
    if(buttonState == HIGH){//Checks if button is pressed
      if(buttonStatus == true){
        buttonStatus = false;
        break;
      }
    }
    
    // sets how long delay is betweeen the frequency changes 
    // ! Changing this will affect the respiratory rate
    delay(100);
  }

  }
  movementDone();
}
