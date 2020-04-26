#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// ----------- VIBRATION MOTORS -----------

const int vibration_motor = 9;  // Vibration motor, pin 9

// ----------- ACCELEROMETER --------------
Adafruit_BNO055 bno = Adafruit_BNO055();

// ----------- LED LIGHTS -----------------
const int green_light_pin_0 = 11;
const int blue_light_pin_0 = 10;
const int green_light_pin_1 = 9;
const int blue_light_pin_1 = 8;
const int green_light_pin_2 = 7;
const int blue_light_pin_2 = 6;
const int green_light_pin_3 = 5;
const int blue_light_pin_3 = 4;

const int brightness = 50; // Comfrotable value for brightness to use with the LEDs

// ----------- BUTTON ---------------------
const int buttonPin = 2; 
int buttonState = 0;
bool buttonStatus = false;


/* Set the delay between fresh samples
 *  #define BNO055_SAMPLERATE_DELAY_MS (100)//This is rate accelerometer 
 *  takes samples, delay in vibration should do the same, not tested.
*/

// Parameters for BREATHING INSTRUCTIONS
 /*  minFrq: There is a pause after each decrease, before the next breath starts.
 *  It happens probably because the vibration frequency values get so low that the motor
 *  cannot produce it anymore. This is why we are starting the fading from a higher value than 0.
 *
 *  fadeStep: FadeStep determines the pace for moving from minFrq value to max value of 255.
 *
 *  Aimed respiratory rate is controlled with the fadeStep and pause values. We can calculate
 *  the respiratory rate by:
 *  R = 60s / ((255-minFrq / fadeStep * a * b * c) + pauseTime * 2)
 *  Where (a * b * c = 0.2)
 *  a = 100ms, delay between steps of vibration frequency
 *  b = 2, because we want to count both inhale and exhale
 *  c = 0.001 because we want to result in seconds instead of millisecond
 */
const int respiratoryRate = 6;        // Targeted respiratory rate
const int pause = 200;                // The break between inhale and exhale (milliseconds)
const int minFrq = 50;                // Min value of vibration frequency
const int fadeStep = (255-minFrq)/((60/respiratoryRate + pause*0.001)/0.2);


// Parameters for MOVEMENT INPUT
float movement[] = {0,0,0};
//int counter = 0;
float filterOne = 0.2;  //filters for movement Y
float filterTwo = 0.4;  //filters for movement Z
float movementVal = 1;  //filter for total amount for movement to be determined
boolean directionDetermined = false;
String direction = "";
int directionInt = -1;
bool directionDone = true;

// Parameters for MOVEMENT INSTRUCTIONS AND FEEDBACK
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


/* FUNCTION:  accelFunc()
 *
 *  function for accelerometer – input and instructions
 *
*/
void accelFunc(){

  /* Set next direction for movement instructions */
  if (directionDone) {

    axis = rand() % 2; // Pick random axis

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

  /* Setting the LEDs for giving the movement instructions */
  if (directionDone == false && directionDetermined == true && insGiven == false) {

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

  /* Receiving movement data from accelerometer */
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float movX = acc.x();
  float movY = acc.y();
  float movZ = acc.z();

  /* Display the floating point data on monitor */
  Serial.print("Acc X: ");
  Serial.print(movX);
  Serial.print(" Y: ");
  Serial.print(movY);
  Serial.print(" Z: ");
  Serial.print(movZ);
  Serial.print("\t");

  // Filter smaller movements
  if(filterOne < movX || movX < - filterOne)movement[0] = movement[0] + movX;
  if(filterOne < movY || movY < - filterOne)movement[1] = movement[1] + movY;
  if(filterTwo < movZ || movZ< - filterTwo)movement[2] = movement[2] + movZ;

  /*  Determine the movement direction to give feedback for
   *  Y = left, -Y = right, Z = up, -Z = down
  */
  if (!directionDetermined) {

    if(movement[1] > movementVal ) {
      direction = "Left";
      directionInt = 3;
      directionDetermined = true;
    }
    else if(movement[1] < -movementVal) {
      direction = "Right";
      directionInt = 2;
      directionDetermined = true;
    }
    else if(movement[2] > movementVal ) {
      direction = "Down";
      directionInt = 1;
      directionDetermined = true;
    }
    else if(movement[2] < -movementVal) {
      direction = "Up";
      directionInt = 0;
      directionDetermined = true;
    }

    /* Check if movement is same as instructed */
    if (directionDetermined == true && feedGiven == false) {

      Serial.print("Intended direction: ");
      Serial.print(next_movement);
      Serial.print(" Actual movement: ");
      Serial.print(directionInt);
      
      if(next_movement == directionInt) {

        /* Movement is correct. Set LED colour green. */

        Serial.print(" movement is correct");

        if (next_movement == 0) {
         analogWrite(green_light_pin_0, brightness);
         analogWrite(blue_light_pin_0, 0);
        }
        else if (next_movement == 1) {
          analogWrite(green_light_pin_1, brightness);
          analogWrite(blue_light_pin_1, 0);
        }
        else if (next_movement == 2) {
         analogWrite(green_light_pin_2, brightness);
         analogWrite(blue_light_pin_2, 0);
        }
        else if (next_movement == 3) {
         analogWrite(green_light_pin_3, brightness);
         analogWrite(blue_light_pin_3, 0);
        }

      }
      
      else if ((next_movement == 0 || next_movement == 1) && (directionInt == 0 || directionInt == 1)){

        /* Movement is semi-correct. Set the LED color green. */

        Serial.print(" movement is semi-correct");

        if (next_movement == 0) {
         analogWrite(green_light_pin_0, brightness);
         analogWrite(blue_light_pin_0, 0);
        }
        else if (next_movement == 1) {
          analogWrite(green_light_pin_1, brightness);
          analogWrite(blue_light_pin_1, 0);
        }
        else if (next_movement == 2) {
         analogWrite(green_light_pin_2, brightness);
         analogWrite(blue_light_pin_2, 0);
        }
        else if (next_movement == 3) {
         analogWrite(green_light_pin_3, brightness);
         analogWrite(blue_light_pin_3, 0);
        }

      }

      else if ((next_movement == 2 || next_movement == 3) && (directionInt == 2 || directionInt == 3)) {
        //movement is semi-correct, green light
        Serial.print(" movement is semi-correct");
        if (next_movement == 0) {
         analogWrite(green_light_pin_0, brightness);
         analogWrite(blue_light_pin_0, 0); // Zero blue color
        }
        else if (next_movement == 1) {
          analogWrite(green_light_pin_1, brightness);
          analogWrite(blue_light_pin_1, 0); // Zero blue color
        }
        else if (next_movement == 2) {
         analogWrite(green_light_pin_2, brightness);
         analogWrite(blue_light_pin_2, 0); // Zero blue color
        }
        else if (next_movement == 3) {
         analogWrite(green_light_pin_3, brightness);
         analogWrite(blue_light_pin_3, 0); // Zero blue color
        }
      }
      else {
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

/* FUNCTION movementDone()
 *
 *  function to control movement instructions after a movement is finished
*/
void movementDone() {

  /* Movement input values from user to zero. */
  movement[0] = 0;
  movement[1] = 0;
  movement[2] = 0;
  direction = "";
  directionDetermined = false;
  directionDone = true;
  insGiven = false;
  feedGiven = false;

  /* Set all LEDs off. */
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
  /* initialize the pushbutton pin as an input:*/
  pinMode(buttonPin, INPUT);

  /* initialize the LED lights: 4 RGB LEDs, using 2 of the colors (2 pins/LED)*/
  pinMode(green_light_pin_0, OUTPUT);
  pinMode(blue_light_pin_0, OUTPUT);
  pinMode(green_light_pin_1, OUTPUT);
  pinMode(blue_light_pin_1, OUTPUT);
  pinMode(green_light_pin_2, OUTPUT);
  pinMode(blue_light_pin_2, OUTPUT);
  pinMode(green_light_pin_3, OUTPUT);
  pinMode(blue_light_pin_3, OUTPUT);

  // Printing accelerometer status and data on monitor:
  
  Serial.begin(9600);
  Serial.println("Accel Sensor Raw Data Test"); Serial.println("");

  /* Initialise the accelerometer*/
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

/* MAIN
 *
*/
void loop() {

  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  if(buttonState == HIGH){
    if(buttonStatus == true)buttonStatus = false;
    else buttonStatus = true;
    delay(1000);
  }
  if(buttonStatus){
    
    /*  For loop, controls the breathing instructions and movement instructions during inhale. 
     *  During each round, the frequency level of thevibration motor is set higher, 
     *  according to fadeStep value.
    */
    for (int fadeValue = minFrq; fadeValue <= 255; fadeValue += fadeStep) {

      // Set the vibration frequency value for current loop
      analogWrite(vibration_motor, fadeValue);

      // movement instructions and movement input
      accelFunc();
  
      buttonState = digitalRead(buttonPin);

      //Checks if button is pressed
      if (buttonState == HIGH) {
        if(buttonStatus == true) {
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
    analogWrite(vibration_motor, 0);

    delay(pause);

    /* For loop, controls the breathing instructions and movement instructions during exhale. */
    // fade out from max to min in increments of 5 points:
    for (int fadeValue = 255; fadeValue >= minFrq; fadeValue -= fadeStep) {

      // sets the value (range from 0 to 255):
      analogWrite(vibration_motor, fadeValue);

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
