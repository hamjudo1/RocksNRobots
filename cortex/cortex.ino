#include "rt.h"
#include "neoSupp.h"
#include "state.h"
#include "thinking.h"
boolean waitForConnection = false;
void setupSerial() {
  Serial.begin(115200);
  if ( waitForConnection ) {
    while (! Serial) {
      delay(1);
    }
  }
}
const boolean normalComm = true;
void setup() {
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  setupDebug();
  setupNeoSupp();
  setupSerial();
  if ( normalComm ) {
    setupComm();
  } else {
    //  We have a bypass mode to test the quadcopter hardware with the unmodified code.
    // This will only work if all of the directly connected SPI bus lines are
    // left as inputs. The slave bus to the cortex is always configured that way, so
    // we can use it, but we can configure the other SPI bus.
    spiSlave_init();
  }
  

  setupRangeFinders();
  setupThinking();
}
double loopCnt = 0.0;
extern void pollWatcher();
void loop() {
  pollDebug();
  pollComm();

  for (int n = 0; n < 4; n++) {
    pixel.setPixelColor(n, hsvColor((double)n * 45.0 + loopCnt, 1.0, 1.0));
  }
  //  pollWatcher();
  delay(100);
  pollNeoSupp();

  loopCnt = loopCnt + 1.0;

  pollRangeFinders();
  pollThinking();
  //
}

