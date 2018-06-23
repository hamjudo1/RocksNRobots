#include "neoSupp.h"
#include "state.h"
#include "rt.h"
boolean radioInitialized = false;
int mode;
long longestQuietTime = 0;
int tookTooLongCount = 0;
boolean readyToUpdateNeoPixels = false;
unsigned long nextNeoUpdate = 0;
int stuffQueued = 0;
int stuffUnqueued = 0;
int SPISlaveInterrupts = 0;
int SPISlaveCharactersRead = 0;
int SPISlaveCharactersWritten = 0;
int cortexState = STATE_CORTEX_ALIVE;
boolean showBrainStemLog = false;
boolean showPacketLog = false;
boolean showXn297LLog = false;
boolean unreadPacket = false;
boolean unprocessedPacket = false;
boolean showRanges = false;
int inputNum = 0;
int userNum = 0;
int rangesInMM[5];
int tState;
void pollDebug() {
  pollBlink();
  if  ( Serial.available() ) {
    int letter = Serial.read();

    if ( letter >= 32 && letter <= 126 ) {
      mode = letter;
      if ( mode == 'r' ) {
        Serial.println("Restarting the radio.");
        radioDefault(); // reset the radio
      } else if ( mode == 's' ) {  // Status
        Serial.print("Longest quiet time: ");
        Serial.println(longestQuietTime);
        Serial.print("Count of take too long events ");
        Serial.println(tookTooLongCount);
        Serial.print("Queue stats queued: ");
        Serial.print(stuffQueued);
        Serial.print(", unqueued: ");
        Serial.println(stuffUnqueued);
        if ( radioInitialized ) {
          Serial.println("Radio initialized");
        } else {
          Serial.println("Radio NOT initialized");
        }
      } else if ( mode == 'u' ) { // Update Neo Pixels
        Serial.println("Updating NeoPixels.");
        quietTime();
      } else if ( mode == 'i' ) { // Init
        Serial.println("Initializing slave SPI");
        spiSlave_init();
      } else if ( mode == 'b' ) {
        Serial.println("Turning off brainstem log listing");
        showBrainStemLog = false;
      } else if ( mode == 'B' ) {
        Serial.println("Turning on brainstem log listing");
        showBrainStemLog = true;
      } else if ( mode == 'x' ) {
        Serial.println("Turning off xn297L log listing");
        showXn297LLog = false;
      } else if ( mode == 'X' ) {
        Serial.println("Turning on xn297L log listing");
        showXn297LLog = true;
      } else if ( mode == 'p' ) {
        Serial.println("Turning off packet log listing");
        showPacketLog = false;
      } else if ( mode == 'P' ) {
        Serial.println("Turning on packet log listing");
        showPacketLog = true;
         } else if ( mode == 'd' ) {
        Serial.println("Turning off distance listing");
        showRanges = false;
      } else if ( mode == 'D' ) {
        Serial.println("Turning on Distance listing");
        showRanges = true;
      } else  if ( letter >= '0' && letter <= '9' ) {
        inputNum = inputNum * 10 + (letter - '0' );
      } else  if ( mode == ',' ) {
        userNum = inputNum;
        inputNum = 0;
      } else  if ( mode == '*' ) {
        if ( inputNum > 0 ) {
          Serial.print("You asked to blink pin ");
          Serial.println(inputNum);
          blinkIt(inputNum);
          inputNum = 0;
        } else { 
          Serial.println("To blink an LED on a pin, enter a 1 or 2 digit number followed by *, for example 23* to blink an LED on pin 23");
        }
      } else {
        Serial.println("We are alive!");
      }
    }
  }
}
int blinkingPin = 0;
void blinkIt(int pinNo) {
  
  if (blinkingPin != 0 ) {
    pinMode(blinkingPin,INPUT);
  }
  blinkingPin = pinNo;
  pinMode(blinkingPin,OUTPUT);
  digitalWrite(blinkingPin,HIGH);
  Serial.print("Trying to blink ");
  Serial.println(blinkingPin);
}
void pollBlink() {
    if ( blinkingPin > 0 ) {
      digitalWrite(blinkingPin, (millis() % 200 < 100));
    }
}
void dispPacketLog() {
  Serial.print("Packets ");
  Serial.print(packetrx);
  Serial.print("rx array: ");
  for ( int rxN = 0; rxN < 4; rxN++ ) {
    Serial.print(rx[rxN]);
    if ( rxN < 3 ) {
      Serial.print(", ");
    }
  }

  Serial.print(" aux array: ");
  for (int aN = 0; aN < AUXNUMBER; aN++) {
    Serial.print((int)aux[aN]);
  }
  Serial.println();
  for ( int b = 0; b < 15; b++) {
    Serial.print(rxdata[b]);
    Serial.print(" ");
  }
  Serial.println();
}
void dispXn297LLog() {
  Serial.println("Please make dispXn297LLog() into a real function");
}
void setupDebug() {
  mode = 0;
}

void quietTime() {
  unsigned long startTime, stopTime, deltaTime;
  startTime = micros();
  if ( readyToUpdateNeoPixels ) {
    NeoUpdate();
  }
  stopTime = micros();
  if ( stopTime < startTime ) {
    // clock rolled over.
    stopTime = stopTime + 1000000;
    startTime = startTime + 1000000;
  }
  deltaTime = stopTime - startTime;
  if ( deltaTime > 1000 ) {
    tookTooLongCount++;
  }
  if ( deltaTime > longestQuietTime ) {
    longestQuietTime = deltaTime;
  }
}
void cortexDebug(enum cortex_debug debug_event) {
  switch (debug_event) {
    case packet_in_fifo:
      pixel.setPixelColor(4, pixel.Color(25, 25, 0));
      break;
    case radio_confused:
      pixel.setPixelColor(4, pixel.Color(0, 0, 50));
      break;
    case we_are_bound:
      pixel.setPixelColor(4, pixel.Color(0, 30, 0));
      break;
    case range_display:
      break;
    case cortex_loop:
      break;
    default:
      break;
  }
}
void cortexDebugRange(enum cortex_debug  debug_event, int rangeFinderIndex, int rangeInMM) {

}

