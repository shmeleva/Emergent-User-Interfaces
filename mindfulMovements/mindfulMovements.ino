#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// ----------- VIBRATION MOTORS -----------

const int vibration_motor = 11;  // Vibration motor, pin 9

// ----------- ACCELEROMETER --------------
Adafruit_BNO055 bno = Adafruit_BNO055();

// ----------- LED LIGHTS -----------------
const int green_light_pin_0 = 2; // non PWM
const int blue_light_pin_0 = 4;

const int green_light_pin_1 = 3; // non PWM
const int blue_light_pin_1 = 6;

const int green_light_pin_2 = 5;
const int blue_light_pin_2 = 7; // non PWM

const int green_light_pin_3 = 9;
const int blue_light_pin_3 = 10;

const int brightness = 255; // Comfrotable value for brightness to use with the LEDs

// ----------- BUTTON ---------------------
const int buttonPin = 12; 
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
bool instructionsGiven = false;
bool feedGiven = false;

//directional instuctions
int next_z = 0;
int prev_z = 1;
int next_y = 2;
int prev_y = 3;
  
int next_movement = -1;
int axis = -1;
int temp = -1;

// -----------------------------------------------------------------------

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
  /*Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");*/

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/* FUNCTION:  setNextDirection()
 *  
 *  Set the direction for next movement.
 *  
 *  req: directionDone = true
 */
void setNextDirection() {

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
    Serial.println("Next movement set.");
}

/* FUNCTION:  giveMovementInstruction();
 *  
 *  Function to set the next movement instruction.
 *  
 *  req = directionDone = false
 *  req = directionDetermined == true
 *  req = instructionsGiven == false
*/
void giveMovementInstruction() {
  
  /* Setting the LEDs for giving the movement instructions */
  if (directionDetermined == true && instructionsGiven == false) {

    if (next_movement == 0) {
      analogWrite(blue_light_pin_0, brightness);
    }
    else if (next_movement == 1) {
      analogWrite(blue_light_pin_1, brightness);
    }
    else if (next_movement == 2) {
      analogWrite(blue_light_pin_2, brightness);
    }
    else if (next_movement == 3) {
      analogWrite(blue_light_pin_3, brightness);
    }
    instructionsGiven == true;
  }
  
}

/* FUNCTION:  readMovementInput()
 *
 *  function for accelerometer – input and instructions
 *
*/
void readMovementInput(){

  Serial.println("Reading movement input.");

  /* Receiving movement data from accelerometer */
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float movX = acc.x();
  float movY = acc.y();
  float movZ = acc.z();

  /* Display the floating point data on monitor */
  /*Serial.print("Acc X: ");
  Serial.print(movX);
  Serial.print(" Y: ");
  Serial.print(movY);
  Serial.print(" Z: ");
  Serial.print(movZ);
  Serial.print("\t");*/

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

      Serial.println("Direction is left.");
    }
    else if(movement[1] < -movementVal) {
      direction = "Right";
      directionInt = 2;
      directionDetermined = true;
      Serial.println("Direction is right.");
    }
    else if(movement[2] > movementVal ) {
      direction = "Down";
      directionInt = 1;
      directionDetermined = true;
      Serial.println("Direction is down.");
    }
    else if(movement[2] < -movementVal) {
      direction = "Up";
      directionInt = 0;
      directionDetermined = true;
      Serial.println("Direction is up.");
    }
  }

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  /*Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);*/

}

/* Set green LEDs */
void correctMovementFeedback() {

    // Our lights are not in the PWM pins and thus some of them cannot use the different brightness level
  
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

/* Set blue LEDs */
void neutralMovementFeedback() {
  
    if (next_movement == 0) {
     analogWrite(green_light_pin_0, 0);
     analogWrite(blue_light_pin_0, brightness);
    }
    else if (next_movement == 1) {
      analogWrite(green_light_pin_1, 0);
      analogWrite(blue_light_pin_1, brightness);
    }
    else if (next_movement == 2) {
     analogWrite(green_light_pin_2, 0);
     analogWrite(blue_light_pin_2, brightness);
    }
    else if (next_movement == 3) {
     analogWrite(green_light_pin_3, 0);
     analogWrite(blue_light_pin_3, brightness);
    }
}

/*  FUNCTION  giveMovementFeedback()
 *   
 *  Give feedback based on the instructions and user's movement.
 *  
 *  req:  directionDetermined = true
 *  req:  feedbackGiven = false
*/
void giveMovementFeedback() {

  Serial.print("Intended direction: ");
  Serial.print(next_movement);
  Serial.print(" Actual movement: ");
  Serial.print(directionInt);

  // MOVEMENT CORRECT
  if (next_movement == directionInt) {

    /* Movement is correct. Set LED colour green. */
    Serial.print(" movement is correct");
    correctMovementFeedback();
  }

  // Semi-correct: right axis
  else if ((next_movement == 0 || next_movement == 1) && (directionInt == 0 || directionInt == 1)) {

    /* Movement is semi-correct. Set LED colour green. */
    Serial.print(" movement is semi-correct");
    correctMovementFeedback();
  }

  // Semi-correct: right axis
  else if ((next_movement == 2 || next_movement == 3) && (directionInt == 2 || directionInt == 3)) {

    /* Movement is semi-correct. Set LED colour green. */
    Serial.print(" movement is semi-correct");
    correctMovementFeedback();
  }
  else {
    
    /* Movement is wrong. Set LED colour blue. */
    Serial.print(" movement is wrong");
    neutralMovementFeedback();
  }
  
  feedGiven = true; // TODO: take off?
  Serial.print("\t");
}

/* FUNCTION movementDone()
 *
 *  function to control movement instructions after a movement is finished
*/
void movementDone() {

  Serial.println("Movement done. Resetting values.");
  
  /* Movement input values from user to zero. */
  movement[0] = 0;
  movement[1] = 0;
  movement[2] = 0;
  direction = "";
  directionDetermined = false;
  directionDone = true;
  instructionsGiven = false;
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

/*  FUNCTION  movementLoop()
 *   
 *   The event loop for 
 *   giving movement instructions,
 *   reading movement input, 
 *   and giving feedback based on actual movement.
*/
void movementLoop() {
  
  if (directionDone) {
    setNextDirection();
  }

  if (!directionDone) {
    giveMovementInstruction();
  }

  readMovementInput();
  
  if (directionDetermined == true && feedGiven == false) {  // TODO: take off feedGiven?
    giveMovementFeedback();
  }

}

/*  FUNCTION  buttonPressed() */

void checkButton() {

  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {

    if (buttonStatus == true) {
      buttonStatus = false;
    }
    else {
      buttonStatus = true;
    }
  }
}

/* MAIN
 *
*/
void loop() {

  checkButton();
  if (buttonState == HIGH) delay(1000);
  
  if (buttonStatus) {

    Serial.println("Inhaling, start of movement.");
    
    for (int fadeValue = minFrq; fadeValue <= 255; fadeValue += fadeStep) {
      
      analogWrite(vibration_motor, fadeValue);
      movementLoop();
      delay(100); /* Changing this will affect the respiratory rate*/
      checkButton();
      if (buttonStatus == false) break;
      
    }

    movementDone();
  }

  if (buttonStatus) {

    Serial.println("Exhaling, start of movement.");
    
    // Stop for a while during the peak before exhale instructions start
    analogWrite(vibration_motor, 0);
    delay(pause);

    for (int fadeValue = 255; fadeValue >= minFrq; fadeValue -= fadeStep) {

      analogWrite(vibration_motor, fadeValue);
      movementLoop();
      delay(100); /* Changing this will affect the respiratory rate*/
      checkButton();
      if (buttonStatus == false) break;
      
    }

    movementDone();
  }
  
}
