/* You might already have an idea of what a variable
 is in math. In programming, a variable is a word with
 just use the number 0, instead.
 */
// ******* Different naming conventions between analog to digital signals
// ******* Investigate variables being defined as const vs int
// ******* should the declarations be in void setup or above?

#include "rnr.h"
const int motor1basespeed = 250;
const int motor2basespeed = 210;
const int led = 3;
const int motor1 = 0;      // motor1 connected to analog pin 7?
const int motor2 = 1;      // motor1 connected to analog pin 8?
int ALTIMETER_PIN = 1;  //Downsensor pin location
int FRONT_SENSOR_PIN = 2;  //Frontsensor pin location
int GYRO_SENSOR_PIN = 0;  // Input signal for gyro value
int altimeter_val = 0;  //Declare variable: measured altimeter value in code.  Initialized at 0
int front_sensor_val = 0; //Declare variable: measured front sensor value in code.  Initialized at 0
int gyro_sensor_val = 0; //Declare variable: measured gyro value in code.  Initialized at 0

int basespeed = (motor1basespeed + motor2basespeed) / 2; // This is the average of the two motor speeds
// before altitude correction.
int helispeed = basespeed; // basespeed with altitude correction.
int trim = (motor1basespeed - motor2basespeed) / 2;
int helispeed7 = motor1basespeed;
int helispeed8 = motor2basespeed;
int normhelispeed7 = motor1basespeed;
int normhelispeed8 = motor2basespeed;
long VCCmV = 3500; // current averaged battery voltage in millivolts.
int rudder = 0;  //Declare variable:

int yaw_rate = 0;
int AltGoal = 100; //#Set by user: roughly in cm to determine free flight height off ground
int AltErr; //Declared: Difference between current altimeter_val and AltGoal
int scan = 0;  //Declared: Used for RH_LH Scan of Front Sensor
int scanDir = 1; //Declared: Used for RH_LH Scan of Front Sensor
int scan2 = 0; //Declared: Used for RH_LH Scan of Front Sensor
int scan2Dir = 1; //Declared: Used for RH_LH Scan of Front Sensor
int counter = 0; //Declared: Used for looping exercises within the code

void setup()  {
  pinMode(ALTIMETER_PIN, INPUT); //sets the pin as input for downward sensor
  pinMode(FRONT_SENSOR_PIN, INPUT); //sets the pin as input for front sensor
  pinMode(GYRO_SENSOR_PIN, INPUT); //sets the pin as input for onboard gyro sensor
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);    // sets the pin as output as RED led out
  pinMode(motor1, OUTPUT);   // sets the pin as output
  pinMode(motor2, OUTPUT);   // sets the pin as output
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
    digitalWrite(led, LOW);
  } else {
    digitalWrite(led, HIGH);
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

  digitalWrite(led, LOW);
  rampMotors(0, basespeed, 50);
  int base_gyro_val = analogRead(GYRO_SENSOR_PIN);
  int gyro_target = base_gyro_val;
  int real_gyro_target = base_gyro_val;

  delay(100);

  while (1) {
    VCCmV = ((VCCmV * 95) / 100);
    VCCmV = (VCCmV + (readVcc() / 100) * 5);

    int altimeter_val = 23000 / (55 + analogRead(ALTIMETER_PIN));
    int front_sensor_val = 7800 / (1 + analogRead(FRONT_SENSOR_PIN));
    int gyro_sensor_val = analogRead(GYRO_SENSOR_PIN);

    AltErr = (AltGoal - altimeter_val) * 3;

    if (AltErr > 30) {
      AltErr = 30;
    }
    if (AltErr < -20) {
      AltErr = -20;
    }

    helispeed = basespeed + AltErr;

    int cycle1Pos = counter % 1000;
    if ( cycle1Pos > 499 ) {
      scanDir = -1;
    } else {
      scanDir = 1;
    }
    // cycle2Pos goes from 0 to 400, then back down to 0. And repeats.
    // cycle2 is a different length than cycle1 so the behavior is more interesting.
    int cycle2Pos = counter % 800;
    if ( cycle2Pos > 400 ) {
      cycle2Pos = 800 - cycle2Pos;
    }
    // Let's make this a nice -100 to +100 value.
    int cycle2Norm = (cycle2Pos - 200) / 2;
    if (front_sensor_val < 80)
    {
      // digitalWrite(5, HIGH);
      gyro_target = base_gyro_val + scanDir * (80 - front_sensor_val) / 5;
    }
    else
    {
      gyro_target = base_gyro_val + cycle2Norm / 20;
    }

    // This next bit of code limits the slew rate, or rate of change.
    if ( gyro_target > real_gyro_target ) {
      real_gyro_target = real_gyro_target + 1;
    } else {
      real_gyro_target = real_gyro_target - 1;
    }
    //yaw_rate = (((base_gyro_val+rudder+scan/2) - gyro_sensor_val));
    yaw_rate = (gyro_sensor_val - real_gyro_target);
    setMotors(helispeed, yaw_rate / 30);

    counter = counter + 1;
    if (counter > 3000) {
      rampMotors(helispeed, helispeed / 2, 300);
      analogWrite(motor1, 0);
      analogWrite(motor2, 0);
      delay(10000);
      counter = 0;
    }
    if (VCCmV < 2300) {
      analogWrite(motor1, 0);
      analogWrite(motor2, 0);
      delay(10000);
    }
    //delay(200);
  }
}
