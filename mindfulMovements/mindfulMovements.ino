#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ----------- VIBRATION MOTORS -----------

const int vibration_motor = 11;  // Vibration motor, pin 11

// ----------- ACCELEROMETER --------------
Adafruit_BNO055 bno = Adafruit_BNO055();

// ----------- LED LIGHTS -----------------

// Up
const int green_light_pin_0 = 2; // non PWM pin
const int blue_light_pin_0 = 4;

// Down
const int green_light_pin_1 = 3; // non PWM pin
const int blue_light_pin_1 = 6;

// Right
const int green_light_pin_2 = 5;
const int blue_light_pin_2 = 7; // non PWM pin

// Left
const int green_light_pin_3 = 9;
const int blue_light_pin_3 = 10;

/*  Some of the LEDs are not in the PWM pins. (Not enough of them for this use).
 *  Thus we cannot use other values than HIGH (255) or LOW (0)
*/
const int brightness = 255; // Comfrotable value for brightness to use with the LEDs

// ----------- BUTTON ---------------------
const int buttonPin = 12; 
int buttonState = 0;
bool buttonStatus = false;


/* Set the delay between fresh samples
 *  #define BNO055_SAMPLERATE_DELAY_MS (100)//This is rate accelerometer 
 *  takes samples, delay in vibration should do the same, not tested.
*/

// Parameters for PHASE 1: WARM-UP
/* Numbers are the amount of full breaths (inhale+exhale) -> 2 movements total */
// TODO: count estimates based on respiratory rate

int startOfExercise = true; // TODO: redundant?
const int warmupLength = 10;
int warmupCounter = 0;
const int exerciseLength = 100;
int exerciseCounter = 0;
bool warmupPhase = true;

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
String directionInput = "";
String directionOutput = "";
int userDirection = -1;
bool directionDone = true;

// Parameters for MOVEMENT INSTRUCTIONS AND FEEDBACK
int directionsDetermined[] = {0,0,0,0}; // amount of directions determined for each direction {up, down, right, left}
const int directionStrings[] = {"Up","Down","Right","Left"};

//directional instuctions
int next_z = 0; // up
int prev_z = 1; // down
int next_y = 2; // right
int prev_y = 3; // left
  
int instructedDirection = -1;
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
      instructedDirection = next_z;
      // swap the directions so next coming is the opposite
      temp = prev_z;
      prev_z = next_z;
      next_z = temp;
    }
    // y-axis
    else {
      instructedDirection = next_y;
      // swap the directions so next coming is the opposite
      temp = prev_y;
      prev_y = next_y;
      next_y = temp;
    }
    
    directionDone = false;

    if (instructedDirection == 0) {
      directionOutput = "Up";

    } else if (instructedDirection == 1) {
      directionOutput = "Down";

    } else if (instructedDirection == 2) {
      directionOutput = "Right";

    } else if (instructedDirection == 3) {
      directionOutput = "Left";

    } else {
      directionOutput = "";
    }

    Serial.println("Next movement set: "); Serial.println(directionOutput);
}

/* FUNCTION:  giveMovementInstruction();
 *  
 *  Function to set the next movement instruction.
 *  
 *  req = directionDone = false
 *  
*/
void giveMovementInstruction() {
  
  /* Setting the LEDs for giving the movement instructions */
  if (instructedDirection == 0) {
    analogWrite(blue_light_pin_0, brightness);
  }
  else if (instructedDirection == 1) {
    analogWrite(blue_light_pin_1, brightness);
  }
  else if (instructedDirection == 2) {
    analogWrite(blue_light_pin_2, brightness);
  }
  else if (instructedDirection == 3) {
    analogWrite(blue_light_pin_3, brightness);
  }
}

/* FUNCTION:  readMovementInput()
 *
 *  function for accelerometer – input and instructions
 *
*/
void readMovementInput(){

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

  if (movement[1] > movementVal) {
    directionDetermined = true;
    directionsDetermined[3] += 1; // add to list as movement
    movement[1] = 0; // reset so new movement may come
    Serial.println("Moving left.");
  }
  else if (movement[1] < -movementVal) {
    directionDetermined = true;
    directionsDetermined[2] += 1;
    movement[1] = 0;
    Serial.println("Moving right.");
  }
  else if (movement[2] > movementVal ) {
    directionDetermined = true;
    directionsDetermined[1] += 1;
    movement[2] = 0;
    Serial.println("Moving down.");
  }
  else if (movement[2] < -movementVal) {
    directionDetermined = true;
    directionsDetermined[0] += 1;
    movement[2] = 0;
    Serial.println("Moving up.");
  }
  else {
    //Serial.println("Could not determine direction.");
  }

  // Determine direction from largest number of movements

  /*int ups = directionsDetermined[0];
  int downs = directionsDetermined[1];
  int rights = directionsDetermined[2];
  int lefts = directionsDetermined[3];
  
  if (ups >= downs && ups >= rights && ups >= lefts) {
    
    directionInput = "Up";
    userDirection = 0;
  }
  else if (downs >= ups && downs >= rights && downs >= lefts) {
    
    directionInput = "Down";
    userDirection = 1;
  }
  else if (rights >= ups && rights >= downs && rights >= lefts) {
    
    directionInput = "Right";
    userDirection = 2;
  }
  else if (lefts >= ups && lefts >= downs && lefts >= rights) {
    directionInput = "Left";
    userDirection = 3;
  }
  else {
    //Serial.println("no direction found with this, check for logic error") //no direction found with this logic
  }*/

  int largestValue = 0;
  for (int i = 0; i < 4; ++i) {

    if (largestValue < directionsDetermined[i]) {

      largestValue = directionsDetermined[i];
      directionInput = directionStrings[i];
      userDirection = i;
    }
  }

  /* Display calibration status for each sensor. */
  /*uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
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
  
    if (instructedDirection == 0) {
      analogWrite(green_light_pin_0, brightness);
      analogWrite(blue_light_pin_0, 0);
    }
    else if (instructedDirection == 1) {
      analogWrite(green_light_pin_1, brightness);
      analogWrite(blue_light_pin_1, 0);
    }
    else if (instructedDirection == 2) {
      analogWrite(green_light_pin_2, brightness);
      analogWrite(blue_light_pin_2, 0);
    }
    else if (instructedDirection == 3) {
      analogWrite(green_light_pin_3, brightness);
      analogWrite(blue_light_pin_3, 0);
    }
}

/* Set blue LEDs */
void neutralMovementFeedback() {
  
    if (instructedDirection == 0) {
      analogWrite(green_light_pin_0, 0);
      analogWrite(blue_light_pin_0, brightness);
    }
    else if (instructedDirection == 1) {
      analogWrite(green_light_pin_1, 0);
      analogWrite(blue_light_pin_1, brightness);
    }
    else if (instructedDirection == 2) {
      analogWrite(green_light_pin_2, 0);
      analogWrite(blue_light_pin_2, brightness);
    }
    else if (instructedDirection == 3) {
      analogWrite(green_light_pin_3, 0);
      analogWrite(blue_light_pin_3, brightness);
    }
}

/*  FUNCTION  giveMovementFeedback()
 *   
 *  Give feedback based on the instructions and user's movement.
 *  
 *  req:  directionDetermined = true
 *  
*/
void giveMovementFeedback() {

  Serial.print("Intended direction:"); 
  Serial.print(instructedDirection); Serial.print("");
  Serial.print("Actual movement:");
  Serial.print(userDirection); Serial.print("");
  Serial.print("Feedback: ");
  
  // MOVEMENT CORRECT
  if (instructedDirection == userDirection) {

    /* Movement is correct. Set LED colour green. */
    Serial.print("movement is correct.");
    correctMovementFeedback();
  }

  // Semi-correct: correct axis
  else if ((instructedDirection == 0 || instructedDirection == 1) && (userDirection == 0 || userDirection == 1)) {

    /* Movement is semi-correct. Set LED colour green. */
    Serial.print("movement is semi-correct.");
    correctMovementFeedback(); // using this since only 2/3 colours in use.
  }

  // Semi-correct: correct axis
  else if ((instructedDirection == 2 || instructedDirection == 3) && (userDirection == 2 || userDirection == 3)) {

    /* Movement is semi-correct. Set LED colour green. */
    Serial.print("movement is semi-correct.");
    correctMovementFeedback(); // using this since only 2/3 colours in use.
  }
  else {
    
    /* Movement is wrong. Set LED colour blue. */
    Serial.print("movement is wrong.");
    neutralMovementFeedback();
  }
  
  Serial.print("");
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
  directionInput = "";
  directionDetermined = false;
  directionDone = true;
  directionsDetermined[0] = 0;
  directionsDetermined[1] = 0;
  directionsDetermined[2] = 0;
  directionsDetermined[3] = 0;

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

/*  FUNCTION  endVisualization()
 *
 *   Creates a visualization in the end of the exercise with the LEDs.
*/
void endVisualization() {
  // do some fancy blinking/circle/etc. here.

  // top
  analogWrite(blue_light_pin_0, HIGH);
  delay(500);

  // right
  analogWrite(blue_light_pin_2, HIGH);
  analogWrite(blue_light_pin_0, LOW);
  delay(500);

  // bottom
  analogWrite(blue_light_pin_1, HIGH);
  analogWrite(blue_light_pin_2, LOW);
  delay(500);

  // left
  analogWrite(blue_light_pin_3, HIGH);
  analogWrite(blue_light_pin_1, LOW);
  delay(500);

  /* greens*/

  // top
  analogWrite(green_light_pin_0, HIGH);
  analogWrite(blue_light_pin_3, LOW);
  delay(500);

  // right
  analogWrite(green_light_pin_2, HIGH);
  analogWrite(green_light_pin_0, LOW);
  delay(500);

  // bottom
  analogWrite(green_light_pin_1, HIGH);
  analogWrite(green_light_pin_2, LOW);
  delay(500);

  // left
  analogWrite(green_light_pin_3, HIGH);
  analogWrite(green_light_pin_1, LOW);
  delay(500);

  analogWrite(green_light_pin_3, LOW);

}

/*  FUNCTION  movementLoop()
 *   
 *   The event loop for 
 *   1 giving movement instructions,
 *   2 reading movement input,
 *   3 and giving feedback based on actual movement.
*/
void movementLoop() {
  
  if (directionDone) {
    setNextDirection();
  }

  if (!directionDone) {
    giveMovementInstruction();
  }

  readMovementInput();
  
  if (directionDetermined == true) { 
    giveMovementFeedback();
  }

}

/*   FUNCTION  checkButton()
 *
 *   Checks whether button is being pressed
 *   and updates the buttonStatus value.
*/
void checkButton() {

  buttonState = digitalRead(buttonPin); // Result will be HIGH during when button is physically pressed down
  if (buttonState == HIGH) {
    /* Wait after button click was detected so we dont keep reading the same button click again. 
     *  This delay does not matter because the button press will always have an effect on the exercise 
     *  -> it will be stopped or it is only starting.
    */
    delay(1000);
    if (buttonStatus == true) {
      buttonStatus = false;
    }
    else {
      buttonStatus = true;
    }
  }
}

/* FUNCTION resetExercise()
 * 
 * Resets warmup and exercise counters when button is pressed or exercise ends
 * 
 */
void resetExercise() {
  warmupCounter = 0; // counting this until warmupLength
  exerciseCounter = 0; // counting this until exerciseLength
  startOfExercise = true;
  warmupPhase = true;
}
 
/* MAIN
 *
*/
void loop() {

  checkButton();
  if (startOfExercise && buttonStatus) {

    Serial.print("Starting exercise."); Serial.println("");
    startOfExercise = false;
    warmupPhase = true; // this is redundant here really

    // TODO: You can play nice led show here. Otherwise this block is useless.
  }

  // TODO: change the "buttonStatus name"
  // TODO: also check the need for all buttonchecks

  if (buttonStatus) {

    for (int fadeValue = minFrq; fadeValue <= 255; fadeValue += fadeStep) {

      /* This sets the vibration motor frequency, 
      changes by the amount of fadeStep parameter */
      analogWrite(vibration_motor, fadeValue);

      /* Here we give the movement instructions, 
      check user's movement direction and give feedback */
      if (warmupPhase == false) movementLoop(); 

      /* Randomly checking whether the button is being pressed down currently */
      checkButton();

      if (buttonStatus == false) {
        /* Button was pressed down. Stopping the exercise 
        and resetting all parameters here. */
        analogWrite(vibration_motor, 0);
        if (warmupPhase == false) endVisualization();
        startOfExercise = true;
        resetExercise();
        break; // if button press is detected, stopping the exercise middle of breathing instructions
      }

      /* Short break to make the vibration frequency changes visible. 
       *  Changing this will affect the respiratory rate. */
      delay(100);
    }

    if (warmupPhase == false) movementDone();
  }

  if (buttonStatus) {

    // Stop for a while during the peak before exhale instructions start
    analogWrite(vibration_motor, 0);
    delay(pause);

    for (int fadeValue = 255; fadeValue >= minFrq; fadeValue -= fadeStep) {

      analogWrite(vibration_motor, fadeValue);
      if (warmupPhase == false) movementLoop();
      delay(100); /* Changing this will affect the respiratory rate*/

      checkButton();
      if (buttonStatus == false) {
        analogWrite(vibration_motor, 0);
        if (warmupPhase == false) endVisualization();
        startOfExercise = true;
        resetExercise();
        break; // if button press is detected, stopping the exercise middle of breathing instructions
      }

    }
    
    if (warmupPhase == true) {
      warmupCounter += 1;
    }
    else {
      /* Resetting all values regarding movement instruction, 
      detection and feedback, getting ready for next movement. */
      movementDone();
      exerciseCounter += 1;
    }
  }

  /*
   *  ENDING THE WARMUP
  */
  if (warmupCounter >= warmupLength) {
    
    warmupPhase = false;
    delay(2000); // break between the warmup and main exercise
    // TODO: could play some LED vis
  }
  /*
   *  ENDING THE MAIN EXERCISE
  */  
  if (exerciseCounter >= exerciseLength) {
    
    buttonStatus = false; // this takes care that we are not doing the breathing/movement instructions if exercisecounter is maxed out!
    analogWrite(vibration_motor, 0);
    endVisualization();
    startOfExercise = true;
    resetExercise();
  }
}
