#include "Arduino.h"
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
void rnrDone() {
  while ( readVcc() > 4000 ) {
    digitalWrite(3,HIGH);
    delay(100);
    digitalWrite(3,LOW);
    delay(100);
  }
  while (1) {
    digitalWrite(3,HIGH);
    delay(100);
    digitalWrite(3,LOW);
    delay(900);
  }
}