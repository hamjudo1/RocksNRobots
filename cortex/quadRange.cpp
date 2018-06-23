#include "Adafruit_VL53L0X.h"
#include <Adafruit_NeoPixel.h>
// i2c pin 31 SDA Arduino20 (default for wire library)
// i2c pin 32 SCL Arduino21 (likewise)

#include "neoSupp.h"
#include "state.h"
Adafruit_VL53L0X loxes[] = {Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(),
                            Adafruit_VL53L0X()
                           };

#define TCAADDR 0x74
// J5 1 CPU pin 12 Port PA07 Arduino Pin 9.
// J5 2 CPU pin 11 Port PA06 Arduino Pin 8.
// J5 3 CPU pin 10 Port PA05 Arduino Pin 18.
// J5 4 CPU pin 9 Port PA04 Arduino Pin 17.
// J5 5 CPU pin 8 Port PB09 Arduino Pin 16.
const int J5X1 = 9;
const int J5X2 = 8;
const int J5X3 = 18;
const int J5X4 = 17;
const int J5X5 = 16;
int8_t J5Apins[5] = { J5X1, J5X2, J5X3, J5X4, J5X5 };


rangeConfigElem_t rangeConfig[] = {
  { -1, true, "right", 0},
  {0, true, "up", 0 },
  {1, true, "left", 0},
  {2, true, "down", 0},
  {3, true, "forward", 0 },
};

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

int activeRangeFinderCnt = 0;
int activeRF[5];
const char * allNames[5];

void J5test() {
  int pinNo = 4;
  pinMode(J5Apins[pinNo], OUTPUT);
  while (true) {
    digitalWrite(J5Apins[pinNo], HIGH);
    delay(2000);
    Serial.print("On pin ");
    Serial.println(J5Apins[pinNo]);
  }
  while (true) {
    for ( int j = 0; j < 6; j++) {
      for (int i = 0; i < j; i++) {
        pinMode(J5Apins[i], OUTPUT);
        digitalWrite(J5Apins[i], HIGH);
      }
      delay(200);
      for (int i = 0; i < 5; i++) {
        digitalWrite(J5Apins[i], LOW);
      }
      delay(200);
    }
  }
}
void setupRangeFinders() {
 // J5test();
  Wire.begin();
  int i, j5PinNo;
  for (i = 0; i < 5; i++) {
    rangesInMM[i] = 0.0;
    j5PinNo = rangeConfig[i].j5Index;
    if ( j5PinNo >= 0 ) {                 // Always set the xshut pin to low output, even for disabled range finders. Or they block the bus.
      pinMode(J5Apins[j5PinNo], OUTPUT);
      digitalWrite(J5Apins[j5PinNo], LOW);
    }
  }
  for (i = 0; i < 5; i++) {
    if ( rangeConfig[i].enabled ) {
      j5PinNo = rangeConfig[i].j5Index;
      if ( j5PinNo >= 0 ) {
        digitalWrite(J5Apins[j5PinNo], HIGH);
      }
      loxes[activeRangeFinderCnt].begin(1 + (i * 2), true);
      allNames[activeRangeFinderCnt] = rangeConfig[i].Name;
      activeRangeFinderCnt++;
    }
  }
}
void pollRangeFinders() {
  VL53L0X_RangingMeasurementData_t measure;

  // Serial.print(F("Reading a measurement... "));

  for (uint8_t loxIndex = 0; loxIndex < activeRangeFinderCnt; loxIndex++) {
    loxes[loxIndex].getSingleRangingMeasurement(&measure, false);
    // lox->rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    if ( showRanges ) {
      Serial.print(" "); Serial.print(allNames[loxIndex]);
    }
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      rangesInMM[loxIndex] = measure.RangeMilliMeter;
      if ( showRanges ) {
        Serial.print(" "); Serial.print(measure.RangeMilliMeter);
      }
    } else {
      rangesInMM[loxIndex] = 1300;
      if ( showRanges ) {
        Serial.print(F(" too far "));
      }
    }
  }
  if ( showRanges ) {
    Serial.println();
  }
}

