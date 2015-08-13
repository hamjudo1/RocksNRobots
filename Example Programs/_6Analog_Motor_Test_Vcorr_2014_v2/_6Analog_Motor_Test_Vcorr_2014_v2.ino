/* You might already have an idea of what a variable 
is in math. In programming, a variable is a word with 
just use the number 0, instead.
*/


/* Not working right. DigiKeyboard does not work at the same time as the red LED on the board.
*/
 //#include "DigiKeyboard.h"
 
 /*take note that motor1 is the bottom rotor, and motor2 is the upper rotor.
   having the upper rotor (motor2) moving faster than the other will cause the heli
   to spin counterclockwise. Likewise, having the bottom rotor, operated by motor1
   moving faster, will make the heli spin clockwise.*/
 
const int led = 3;
const int motor1 = 0;      // motor1 connected to analog pin 7
const int motor2 = 1;      // motor1 connected to analog pin 8

int motor1basespeed = 210; // these numbers will need to be tweaked for your
int motor2basespeed = 170; // specific heli - the default value is 135 for both.

int normmotor1speed;
int normmotor2speed;
long VCCmV = 3500;



void setup()  { 
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);    // sets the pin as output
  pinMode(motor1, OUTPUT);   // sets the pin as output
  pinMode(motor2, OUTPUT);   // sets the pin as output

} 

void loop()  { 
  
  for(int counter = 0; counter < 300; counter++)
  {
    VCCmV = ((VCCmV*95)/100); 
    VCCmV = (VCCmV + (readVcc()/100)*5);
    
    normmotor1speed =int(((long(motor1basespeed))*long(1330))/(VCCmV-2025));    
    normmotor2speed =int(((long(motor2basespeed))*long(1330))/(VCCmV-2025));     
    
    analogWrite(motor1, normmotor1speed); 
    analogWrite(motor2, normmotor2speed);   
   
    
    digitalWrite(led, HIGH);

    delay(10);
  }
  
  for(int counter = 0; counter <= 100; counter = counter++)
  {
    analogWrite(motor1, normmotor1speed-counter);  // gradually slows motors, keeping heli from plummeting,
    analogWrite(motor2, normmotor2speed-counter);  // which you just hate to see it do every time
    delay(20);
  }  
  
  analogWrite(motor1, 0); 
  analogWrite(motor2, 0);
  digitalWrite(led, LOW);
  delay(10000);  
}
long readVcc() {
  // Source: http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    //DigiKeyboard.println(1);
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     //DigiKeyboard.println(2);
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
     //DigiKeyboard.println(3);
     ADMUX = _BV(MUX3) | _BV(MUX2);   
  #else
    //DigiKeyboard.println(4);
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
  
  long result = (high<<8) | low;
 //DigiKeyboard.println(result);8

  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  result = 1211861L / result; 
  return result; // Vcc in millivolts
}

