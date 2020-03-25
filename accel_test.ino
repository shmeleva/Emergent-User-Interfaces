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

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
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
boolean hasMovement = false;
int counter = 0;

void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
 

  

  /* Display the floating point data */
  float movX = acc.x();
  float movY = acc.y();
  float movZ = acc.z();
  /*Serial.print("Acc X: ");
  Serial.print(movX);
  Serial.print(" Y: ");
  Serial.print(movY);
  Serial.print(" Z: ");
  Serial.print(movZ);
  Serial.print("\t");*/

  
  //X = front, -X = back, Y = left, -Y = right, Z = up, -Z = down, 
  //Checks if movement has ended
  if(hasMovement && abs(movX)<0.15 && abs(movY)<0.15 && abs(movZ)<0.30){
    if(abs(movement[0])>abs(movement[1])&&abs(movement[0])>abs(movement[2])){
      if(movement[0]>0)Serial.print("Front");
      else Serial.print("Back");
    }
    else if(abs(movement[1])>abs(movement[2])){
      if(movement[1]>0)Serial.print("Left");
      else Serial.print("Right");
    }
    else{
      if(movement[2]>0)Serial.print("Up");
      else Serial.print("Down");
    }
    Serial.print("\t");
    hasMovement = false;
    movement[0] = 0;
    movement[1] = 0;
    movement[2] = 0;
    delay(3000);
  }

  //Resets counter so movement wont be detected from static state over time
  /*counter = counter + 1;
  if(counter>10){
    counter = 0;
    movement[0] = 0;
    movement[1] = 0;
    movement[2] = 0;
  }*/

  //Filters smaller movements
  if(0.15 < movX || movX < -0.15)movement[0] = movement[0] + movX;
  if(0.15 < movY || movY < -0.15)movement[1] = movement[1] + movY;
  if(0.30 < movZ || movZ< -0.30)movement[2] = movement[2] + movZ;

  //Checks if movement is large enough
  if(abs(movement[0])> 1 || abs(movement[1]) > 1 || abs(movement[2]) > 1){
    hasMovement = true; 
  }

  Serial.print("testi: ");
  Serial.print(movement[0]);
  Serial.print(" ");
  Serial.print(movement[1]);
  Serial.print(" ");
  Serial.print(movement[2]);
  Serial.print("\t");


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
