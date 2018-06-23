#include "neoSupp.h"
#include "rt.h"
#include "thinking.h"
#include "state.h"
const int DOWNRANGE = 3;


void setupThinking() {
  tState = 0;
}
void pollThinking() {
  boolean changedIt = false;
  if ( unreadPacket ) {
    processPacket(XN297L_payloadIn[XN297L_goodPayloadIn]);
    if ( tState == 0 ) { // wait to initialize.
      if ( millis() > 5000 ) {
        tState = 1; // Scheduled takeoff time.
        Serial.println("Not time for flight");
      } else {
        Serial.println("Takeoff time, check the checklist");
      }
    } else if ( tState == 1 ) { // Pre flight checklist
      // Check battery voltage

      // Check for valid connection from remote

      // Check that downward rangefinder is working
      if ( rangesInMM[DOWNRANGE] > 0 && rangesInMM[DOWNRANGE]  < 100 ) {
        Serial.println("Cleared for flight\n");
        tState = 2;
      } else {
        Serial.print("Wait for good altitude, not ");
        Serial.println(rangesInMM[DOWNRANGE]);
      }

    } else if ( tState == 2 ) {
      float newRx[4];
      for (int i = 0; i < 4; i++) {
        newRx[i] = rx[i];
      }
      if ( rangesInMM[DOWNRANGE] < 300 ) {
        newRx[3] = 0.3;
        Serial.print ("Too low ");

      } else if ( rangesInMM[DOWNRANGE] < 500 ) {
        newRx[3] = 0.2;
         Serial.print ("good height ");
      } else {
        newRx[3] = 0.11;
         Serial.print ("Too high ");
      }
      Serial.println(rangesInMM[DOWNRANGE]);
      replaceRx(newRx, XN297L_payloadOut[!XN297L_goodPayloadOut], XN297L_payloadIn[XN297L_goodPayloadIn]);
      changedIt = true;
    }
    if ( ! changedIt ) { // Copy unmodified packet.
      for (int i = 0; i < 14; i++) {
        XN297L_payloadOut[!XN297L_goodPayloadOut][i] = XN297L_payloadIn[XN297L_goodPayloadIn][i];
      }
    }
    updateChecksum(XN297L_payloadOut[!XN297L_goodPayloadOut]);
    XN297L_goodPayloadOut = !XN297L_goodPayloadOut;
  } else {
    Serial.println("no packet yet");
  }
}

