
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


void setup() { 
}

void loop() {
  
  // fade in from min to max in increments of 5 points:
  for (int fadeValue = minFrq; fadeValue <= 255; fadeValue += fadeStep) {
    
    // sets the value (range from 0 to 255):
    analogWrite(vMotor1, fadeValue);
    analogWrite(vMotor2, fadeValue);

    // sets how long delay is betweeen the frequency changes 
    // ! Changing this will affect the respiratory rate
    delay(100);
  }

  // Stop for a while during the peak before exhale instructions start
  analogWrite(vMotor1, 0);
  analogWrite(vMotor2, 0);

  delay(pause);

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255; fadeValue >= minFrq; fadeValue -= fadeStep) {
    
    // sets the value (range from 0 to 255):
    analogWrite(vMotor1, fadeValue);
    analogWrite(vMotor2, fadeValue);

    // sets how long delay is betweeen the frequency changes 
    // ! Changing this will affect the respiratory rate
    delay(100);
  }
  
}
