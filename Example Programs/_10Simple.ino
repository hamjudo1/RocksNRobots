/* You might already have an idea of what a variable
 is in math. In programming, a variable is a word with
 just use the number 0, instead.
 */

#include "rnr.h"
const int motor1basespeed = 260;
const int motor2basespeed = 220;
const int led = 3;  // LED is on digital pin 3
const int motor1 = 0; // First motor on digital pin 0
const int motor2 = 1;    // Second motor on digital pin 1
int ALTIMETER_PIN = 1;  //Downsensor analog pin location (analog pin 1, is different from digital pin 1!) 
int FRONT_SENSOR_PIN = 2;  //Frontsensor pin location
int GYRO_SENSOR_PIN = 0;  // Input signal for gyro value (analog pin 0 different from digital pin 0!)
int altimeter_val = 0;  //Declare variable: measured altimeter value in code.  Initialized at 0
int front_sensor_val = 0; //Declare variable: measured front sensor value in code.  Initialized at 0
int gyro_sensor_val = 0; //Declare variable: measured gyro value in code.  Initialized at 0

int basespeed = (motor1basespeed + motor2basespeed) / 2; // This is the average of the two motor speeds
// before altitude correction.
int helispeed = basespeed; // basespeed with altitude correction.
int trim = (motor1basespeed - motor2basespeed) / 2;

long VCCmV = 3500; // current averaged battery voltage in millivolts.
int rudder = 0;  //Declare variable:

int yaw_rate = 0;
int AltGoal = 120; //#Set by user: roughly in cm to determine free flight height off ground
int AltErr; //Declared: Difference between current altimeter_val and AltGoal
int counter = 0; //Declared: Used for looping exercises within the code

void setup()  {
  // initialize the digital output pins.
  pinMode(led, OUTPUT);    // sets the pin as output as RED led out
  pinMode(motor1, OUTPUT);   // sets the pin as output
  pinMode(motor2, OUTPUT);   // sets the pin as output
  // The analog input pins don't need to be initialized.
  VCCmV = readVcc();
}

void setMotors(int hSpeed, int yawRate) {
  int mSpeed1 = hSpeed + trim + yawRate;
  int mSpeed2 = hSpeed - trim - yawRate;
  int maxNormSpeed;

  int normSpeed1 = int(((long(mSpeed1)) * long(1330)) / (VCCmV - 2025));
  int normSpeed2 = int(((long(mSpeed2)) * long(1330)) / (VCCmV - 2025));

  // Motor speeds can only go up to 255. If asked for something higher, proportionally reduce both speeds.
  // We actually use 254 in case there is an off by one somewhere or a rounding error.
  if ( normSpeed1 > normSpeed2 ) {
    maxNormSpeed = normSpeed1;
  } else {
    maxNormSpeed = normSpeed2;
  }
  if ( maxNormSpeed > 254 ) {
    normSpeed1 = (normSpeed1 * 254) / maxNormSpeed;
    normSpeed2 = (normSpeed2 * 254) / maxNormSpeed;
   // digitalWrite(led, LOW);
  } else {
   // digitalWrite(led, HIGH);
  }
  // Motor speeds can only go down to 0. Low speeds only happen before liftoff, or after landing, so
  // we won't bother to be proportional. Just make sure it never goes negative.
  if ( normSpeed1 < 0 ) {
    normSpeed1 = 0;
  }
  if (normSpeed2 < 0) {
    normSpeed2 = 0;
  }
  analogWrite(motor1, normSpeed1);
  analogWrite(motor2, normSpeed2);
}
// Ramp the motors from one value to another;
void rampMotors(int startSpeed, int endSpeed, int stepSize) {
  int tenths;
  int hSpeed;
  for ( tenths = 0 ; tenths < 11; tenths++ ) {
    hSpeed = (startSpeed * (10 - tenths) + (endSpeed * tenths)) / 10;
    setMotors(hSpeed, 0);
    delay(stepSize);
  }
}
void loop()  {
  /* Obtaining initial gyro input value under full electrical load conditions. Soley for purposes of getting
  initial gyro value.  USE SPIN TEST ON STUDENTS TO DEMONSTRATE THE GYRO*/

  delay(1000); // 1 second to unplug the battery after programming.
  digitalWrite(led, LOW);
  rampMotors(0, basespeed, 50);
  int base_gyro_val = analogRead(GYRO_SENSOR_PIN);
  int gyro_target = base_gyro_val;

  delay(100);

  while (1) {
    VCCmV = ((VCCmV * 95) / 100);
    VCCmV = (VCCmV + (readVcc() / 100) * 5);

    int altimeter_val = 23000 / (55 + analogRead(ALTIMETER_PIN));
    int front_sensor_val = 23000 / (55 + analogRead(FRONT_SENSOR_PIN));
    int gyro_sensor_val = analogRead(GYRO_SENSOR_PIN);

    AltErr = (AltGoal - altimeter_val) * 3;

    if (AltErr > 30) {
      AltErr = 30;
    }
    if (AltErr < -20) {
      AltErr = -20;
    }

    helispeed = basespeed + AltErr;

    if (front_sensor_val < 100)
    {
      digitalWrite(led, HIGH);
      gyro_target = base_gyro_val - 100;
    }
    else
    {
      digitalWrite(led, LOW);
      gyro_target = base_gyro_val;
    }
    yaw_rate = ( gyro_target - gyro_sensor_val );
    if ( yaw_rate > 30 ) {
      yaw_rate = 30;
    } else if ( yaw_rate < -30 ) {
      yaw_rate = -30;
    }
    setMotors(helispeed, yaw_rate );

    counter = counter + 1;
    if (counter > 3000) {
      rampMotors(helispeed, helispeed / 2, 300); // Gently land.
      analogWrite(motor1, 0); // Turn the motors all the way off.
      analogWrite(motor2, 0);
      delay(10000);
      counter = 0;
    }
    if (VCCmV < 2300) {
      analogWrite(motor1, 0);
      analogWrite(motor2, 0);
      delay(10000);
    }
  }
}
