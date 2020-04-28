#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Variables to customize for the exercise:

const float warmupDuration = 0.5;    // Give the duration for warmup phase in minutes
const float exerciseDuration = 5;  // Give the duration for main exercise phase in minutes
const int respiratoryRate = 7;  // Set the targeted respiratory rate


// ----------- VIBRATION MOTORS -----------

const int vibration_motor = 11;  // Vibration motor, pin 11

// ----------- ACCELEROMETER --------------
Adafruit_BNO055 bno = Adafruit_BNO055();

// ----------- LED LIGHTS -----------------

// Up
const int green_light_pin_0 = 2; // non PWM pin - use digitalWrite() HIGH/LOW
const int blue_light_pin_0 = 4;  // non PWM pin - use digitalWrite() HIGH/LOW

// Down
const int green_light_pin_1 = 3;
const int blue_light_pin_1 = 6;

// Right
const int green_light_pin_2 = 5;
const int blue_light_pin_2 = 7; // non PWM pin - use digitalWrite() HIGH/LOW

// Left
const int green_light_pin_3 = 9;
const int blue_light_pin_3 = 10;

/*  Some of the LEDs are not in the PWM pins. (Not enough of them for this use).
 *  Thus we cannot use other values than HIGH (255) or LOW (0)
*/
const int brightness = 255;

// ----------- BUTTON ---------------------
const int buttonPin = 12; 
int buttonState = 0;
bool continueExercise = false;


/* Set the delay between fresh samples
 *  #define BNO055_SAMPLERATE_DELAY_MS (100)//This is rate accelerometer 
 *  takes samples, delay in vibration should do the same, not tested.
*/


// Parameters for PHASE 1: WARM-UP
/*  Values for these are the amount of full breaths (inhale+exhale) -> 2 movements total. 
 *   
 *  We can estimate the duration of one full loop of the exercise with respiratory rate
 *  since the exercise parameters that increase the duration are counted based 
 *  on the set respiratory rate.
 *  
 *  duration of one round (full breath) = 60 / respiratory rate 
*/
int startOfExercise = true;
bool warmupPhase = false;

const int warmupLength = (warmupDuration*60)/(60/respiratoryRate);
const int exerciseLength = (exerciseDuration*60)/(60/respiratoryRate);

int warmupCounter = 0;
int exerciseCounter = 0;



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
const int pause = 200;                // The break between inhale and exhale (milliseconds)
const int minFrq = 60;                // Min value of vibration frequency
const int fadeStep = (255-minFrq)/((60/respiratoryRate + pause*0.001)/0.2);



// Parameters for MOVEMENT INPUT

float movement[] = {0,0,0};
//int counter = 0;
float filterOne = 0.2;  //filters for movement Y
float filterTwo = 0.4;  //filters for movement Z
float movementVal = 1;  //filter for total amount for movement to be determined
boolean directionDetermined = false;
int userDirection = -1;
bool instructionsGiven = false;
bool directionSet = false;

// Parameters for MOVEMENT INSTRUCTIONS AND FEEDBACK

int directionsDetermined[] = {0,0,0,0}; // amount of directions determined for each direction {up, down, right, left}
const String directionStrings[] = {"Up","Down","Right","Left"};
int latestFeedback = -1;

//directional instuctions
int next_z = 0; // up
int prev_z = 1; // down
int next_y = 2; // right
int prev_y = 3; // left

int nextInstruction = -1;
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

  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/* FUNCTION:  setNextDirection()
 *  
 *  Set the direction for next movement.
 *  
 */
void setNextDirection() {

    axis = rand() % 2; // Pick random axis

    // z-axis
    if (axis == 0) {
      nextInstruction = next_z;
      // swap the directions so next coming is the opposite
      temp = prev_z;
      prev_z = next_z;
      next_z = temp;
    }
    // y-axis
    else {
      nextInstruction = next_y;
      // swap the directions so next coming is the opposite
      temp = prev_y;
      prev_y = next_y;
      next_y = temp;
    }

    String directionOutput = "";
    if (0 <= nextInstruction <= 3) {
      directionOutput = directionStrings[nextInstruction];  
    }
    Serial.print("Upcoming next: "); Serial.println(directionOutput); 
    Serial.println(""); Serial.println("");

    directionSet = true;
}

/* FUNCTION:  giveMovementInstruction();
 *  
 *  Function to set the next movement instruction.
 *  
 *  req = instructionsGiven = false
 *  
*/
void giveMovementInstruction() {
  
  /* Setting the LEDs for giving the movement instructions */
  instructedDirection = nextInstruction;

  /* Give instruction on monitor */
  String directionOutput = "";
  if (0 <= instructedDirection <= 3) {
      directionOutput = directionStrings[instructedDirection];  
  }
  Serial.print("GO – "); Serial.println(directionOutput);
  
  if (instructedDirection == 0) {
    digitalWrite(blue_light_pin_0, HIGH);
    digitalWrite(green_light_pin_0, LOW);
  }
  else if (instructedDirection == 1) {
    digitalWrite(blue_light_pin_1, HIGH);
    digitalWrite(green_light_pin_1, LOW);
  }
  else if (instructedDirection == 2) {
    digitalWrite(blue_light_pin_2, HIGH);
    digitalWrite(green_light_pin_2, LOW);
  }
  else if (instructedDirection == 3) {
    digitalWrite(blue_light_pin_3, HIGH);
    digitalWrite(green_light_pin_3, LOW);
  }

  instructionsGiven = true;
  directionSet = false; // the next instruction is not set yet
}

void determineDirection() {

  // Determine direction from largest number of movements

  /*Serial.println("Current values:"); 
  for (int i = 0; i < 4; ++i) {
    Serial.println(directionsDetermined[i]);
  }*/
  String directionInput = "";
  int largestValue = 0;
  for (int i = 0; i < 4; ++i) {

    if (largestValue < directionsDetermined[i]) {

      largestValue = directionsDetermined[i];
      directionInput = directionStrings[i];

      directionDetermined = true;
      userDirection = i;
      
      /*Serial.println("Determined direction:");
      Serial.println(directionInput);*/
    }
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
    //directionDetermined = true;
    directionsDetermined[3] += 1; // add to list as movement
    movement[1] = 0; // reset so new movement may come
    //Serial.println("Moving left.");
  }
  else if (movement[1] < -movementVal) {
    //directionDetermined = true;
    directionsDetermined[2] += 1;
    movement[1] = 0;
    //Serial.println("Moving right.");
  }
  else if (movement[2] > movementVal ) {
    //directionDetermined = true;
    directionsDetermined[1] += 1;
    movement[2] = 0;
    //Serial.println("Moving down.");
  }
  else if (movement[2] < -movementVal) {
    //directionDetermined = true;
    directionsDetermined[0] += 1;
    movement[2] = 0;
    //Serial.println("Moving up.");
  }
  else {
    //Serial.println("Could not determine direction.");
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
      digitalWrite(green_light_pin_0, HIGH);
      digitalWrite(blue_light_pin_0, LOW);
    }
    else if (instructedDirection == 1) {
      digitalWrite(green_light_pin_1, HIGH);
      digitalWrite(blue_light_pin_1, LOW);
    }
    else if (instructedDirection == 2) {
      digitalWrite(green_light_pin_2, HIGH);
      digitalWrite(blue_light_pin_2, LOW);
    }
    else if (instructedDirection == 3) {
      digitalWrite(green_light_pin_3, HIGH);
      digitalWrite(blue_light_pin_3, LOW);
    }
}

/* Set blue LEDs */
void neutralMovementFeedback() {
  
    if (instructedDirection == 0) {
      digitalWrite(green_light_pin_0, LOW);
      digitalWrite(blue_light_pin_0, HIGH);
    }
    else if (instructedDirection == 1) {
      digitalWrite(green_light_pin_1, LOW);
      digitalWrite(blue_light_pin_1, HIGH);
    }
    else if (instructedDirection == 2) {
      digitalWrite(green_light_pin_2, LOW);
      digitalWrite(blue_light_pin_2, HIGH);
    }
    else if (instructedDirection == 3) {
      digitalWrite(green_light_pin_3, LOW);
      digitalWrite(blue_light_pin_3, HIGH);
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

  //Serial.print("Intended direction: "); 
  //Serial.println(instructedDirection);
  //Serial.print("Actual movement: ");
  //Serial.println(userDirection);

  int thisFeedback = -1;
 
  if (instructedDirection == userDirection)
  {
    // Correct movement
    thisFeedback = 1;
  }
  else if ((instructedDirection == 0 || instructedDirection == 1) && (userDirection == 0 || userDirection == 1))
  {
    // Semi-correct: correct axis
    thisFeedback = 0;
  }
  else if ((instructedDirection == 2 || instructedDirection == 3) && (userDirection == 2 || userDirection == 3))
  {
    // Semi-correct: correct axis
    thisFeedback = 0;
  }
  else 
  {
    // Wrong movement
    thisFeedback = -1;
  }

  /* Change only if feedback changes */
  if (thisFeedback != latestFeedback) {
 
    latestFeedback = thisFeedback;

    if (thisFeedback == 1 || thisFeedback == 0) {
      
      /* Movement is correct or semi-correct. Set LED colour green. */
      Serial.println("Movement CORRECT.");
      correctMovementFeedback();
      
    } else {

      /* Movement is wrong. Set LED colour blue. */
      Serial.println("Movement WRONG.");
      neutralMovementFeedback();
    }
  }
}

/* FUNCTION movementDone()
 *
 *  function to control movement instructions after a movement is finished
*/
void movementDone() {
  
  /* Movement input values from user to zero. */
  Serial.println("Movement finished."); 
  Serial.println(""); Serial.println(""); Serial.println("");
  movement[0] = 0;
  movement[1] = 0;
  movement[2] = 0;
  directionDetermined = false;
  instructionsGiven = false;
  directionsDetermined[0] = 0;
  directionsDetermined[1] = 0;
  directionsDetermined[2] = 0;
  directionsDetermined[3] = 0;

  /* Set all LEDs off. */
  digitalWrite(green_light_pin_0, LOW);
  digitalWrite(blue_light_pin_0, LOW);
  digitalWrite(green_light_pin_1, LOW);
  digitalWrite(blue_light_pin_1, LOW);
  digitalWrite(green_light_pin_2, LOW);
  digitalWrite(blue_light_pin_2, LOW);
  digitalWrite(green_light_pin_3, LOW);
  digitalWrite(blue_light_pin_3, LOW);
       
}

/*  FUNCTION  movementLoop()
 *   
 *   The event loop for 
 *   1 giving movement instructions,
 *   2 reading movement input,
 *   3 and giving feedback based on actual movement.
*/
void movementLoop() {

  /* Set the next instructions already during the previous one. */
  if (!directionSet && instructionsGiven) {
    setNextDirection();
  }

  /* Give instructions once. */
  if (!instructionsGiven && directionSet) {
    giveMovementInstruction();
  }

  readMovementInput();
  determineDirection();
 
  if (directionDetermined == true) { 
    giveMovementFeedback();
    directionDetermined == false;
  }

}

/*   FUNCTION  checkButton()
 *
 *   Checks whether button is being pressed
 *   and updates the continueExercise value.
*/
void checkButton() {

  buttonState = digitalRead(buttonPin); // Result will be HIGH during when button is physically pressed down
  if (buttonState == HIGH) {
    /* Wait after button click was detected so we dont keep reading the same button click again. 
     *  This delay does not matter because the button press will always have an effect on the exercise 
     *  -> it will be stopped or it is only starting.
    */
    delay(1000);
    if (continueExercise == true) {
      continueExercise = false;
    }
    else {
      continueExercise = true;
    }
  }
}

/*  FUNCTION  endVisualization()
 *
 *   Creates a visualization in the end of the exercise with the LEDs.
*/
void endVisualization() {
  // do some fancy blinking/circle/etc. here.

  // top
  digitalWrite(blue_light_pin_0, HIGH);
  delay(200);

  // right
  digitalWrite(blue_light_pin_2, HIGH);
  digitalWrite(blue_light_pin_0, LOW);
  delay(200);

  // bottom
  digitalWrite(blue_light_pin_1, HIGH);
  digitalWrite(blue_light_pin_2, LOW);
  delay(200);

  // left
  digitalWrite(blue_light_pin_3, HIGH);
  digitalWrite(blue_light_pin_1, LOW);
  delay(200);

  /* greens*/

  // top
  digitalWrite(green_light_pin_0, HIGH);
  digitalWrite(blue_light_pin_3, LOW);
  delay(200);

  // right
  digitalWrite(green_light_pin_2, HIGH);
  digitalWrite(green_light_pin_0, LOW);
  delay(200);

  // bottom
  digitalWrite(green_light_pin_1, HIGH);
  digitalWrite(green_light_pin_2, LOW);
  delay(200);

  // left
  digitalWrite(green_light_pin_3, HIGH);
  digitalWrite(green_light_pin_1, LOW);
  delay(200);

  digitalWrite(green_light_pin_3, LOW);

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

/* FUNCTION finishExercise()
 *  
 *  Doing all important settings here when all movements 
 *  are done or when button is pressed to stop the exercise.
 *  
*/
void finishExercise() {

  Serial.println("Finishing exercise.");
  Serial.println("");
  movementDone();
  analogWrite(vibration_motor, 0);
  if (warmupPhase == false) endVisualization();
  startOfExercise = true;
  resetExercise();
}
 
/* MAIN
 *
*/
void loop() {

  checkButton();
  if (startOfExercise && continueExercise) {

    Serial.println("Starting exercise.");
    Serial.println("");

    setNextDirection(); // set first direction
    
    startOfExercise = false;
    warmupPhase = true; // Not redundant!
    
  }
  if (warmupPhase && continueExercise) {
    Serial.print("Warmup round, rounds left: ");
    Serial.println(warmupLength-warmupCounter);
    Serial.println(""); 
  }

  if (continueExercise) {

    for (int fadeValue = minFrq; fadeValue <= 255; fadeValue += fadeStep) {

      /* This sets the vibration motor frequency, 
      changes by the amount of fadeStep parameter */
      analogWrite(vibration_motor, fadeValue);

      /* Here we give the movement instructions, 
      check user's movement direction and give feedback */
      if (warmupPhase == false) movementLoop(); 

      /* Randomly checking whether the button is being pressed down currently */
      checkButton();

      if (continueExercise == false) {
        /* Button was pressed down. Stopping the exercise 
        and resetting all parameters here. */
        finishExercise();
        break;
      }

      /* Short break to make the vibration frequency changes visible. 
       *  Changing this will affect the respiratory rate. */
      delay(100);
    }

    /* Resetting all values regarding movement instruction, 
    detection and feedback, getting ready for next movement. */
    if (warmupPhase == false) movementDone();
  }

  if (continueExercise) {

    /* Pausing shortly before exhale instructions start. */
    analogWrite(vibration_motor, 0);
    delay(pause);

    for (int fadeValue = 255; fadeValue >= minFrq; fadeValue -= fadeStep) {

      analogWrite(vibration_motor, fadeValue);
      if (warmupPhase == false) movementLoop();

      checkButton();
   
      if (continueExercise == false) {
        finishExercise();
        break;
      }

      delay(100);
    }

    analogWrite(vibration_motor, 0);

    /* Counting rounds for warmup and main exercise. */
    if (warmupPhase == true) {
      warmupCounter += 1;
    }
    else {
      movementDone();
      exerciseCounter += 1;
    }
  }

  /*
   *  ENDING THE WARMUP
  */
  if (warmupCounter >= warmupLength && warmupPhase == true && continueExercise) {
    
    warmupPhase = false;
    /* Doing a short pause between the warmup and main exercise starting. */
    Serial.println("Warmup is finished.");
    Serial.println("");
    delay(2000);
    endVisualization();
    
  }
  /*
   *  ENDING THE MAIN EXERCISE
  */  
  if (exerciseCounter >= exerciseLength && continueExercise == true) {

    /* continueExercise variable takes care that we are not doing the breathing/movement 
    instructions if exercisecounter is maxed out! */
    continueExercise = false;
    finishExercise();
  }
}
