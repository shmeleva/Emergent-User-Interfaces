#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();
const int buttonPin = 2; 
int buttonState = 0;
bool buttonStatus = false;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
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

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/


float movement[] = {0,0,0};
//int counter = 0;
float filterOne = 0.2;//filters for movement Y
float filterTwo = 0.4;//filters for movement Z
float movementVal = 1;//filter for total amount for movement to be determined
boolean notDetermined = true;
String direction = "";
int directionInt = -1;

//directional instuctions
int next_z = 0;
int prev_z = 1;
int next_y = 2;
int prev_y = 3;
  
int next_movement = -1;
int axis = -1;
int temp = -1;

bool directionDone = true;



void loop(void)
{
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  if(buttonState == HIGH){
    if(buttonStatus == true)buttonStatus = false;
    else buttonStatus = true;
    delay(2000);
  }
 
  if (buttonStatus == true) {



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

  /*Serial.print("test: ");
  Serial.print(movement[0]);
  Serial.print(" ");
  Serial.print(movement[1]);
  Serial.print(" ");
  Serial.print(movement[2]);
  Serial.print("\t");*/
  
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
    if(notDetermined == false){//Checks if movement is same as instructed
      Serial.print("Intended direction: ");
      Serial.print(next_movement);
      Serial.print(" Actual movement: ");
      Serial.print(directionInt);
      if(next_movement == directionInt){
        //movement is correct
        Serial.print(" movement is correct");
        directionDone = true;
      }
      else if((next_movement == 0 || next_movement == 1) && (directionInt == 0 || directionInt == 1)){
        //movement is semi-correct
        Serial.print(" movement is semi-correct");
        directionDone = true;
      }
      else if((next_movement == 2 || next_movement == 3) && (directionInt == 2 || directionInt == 3)){
        //movement is semi-correct
        Serial.print(" movement is semi-correct");
        directionDone = true;
      }
      else{
        //movement is wrong
        Serial.print(" movement is wrong");
      }
      Serial.print("\t");
    }
  }
  //Checks if movement has ended
  if(notDetermined == false && abs(movX)<0.08 && abs(movY)<0.08 && abs(movZ)<0.21){
    Serial.print("Movement has stopped");
    Serial.print("\t");
    movement[0] = 0;
    movement[1] = 0;
    movement[2] = 0;
    direction = "";
    notDetermined = true;
    delay(2000);
    }


}//end of button
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

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
