#include "neoSupp.h"
#include "state.h"
const long minNeoUpdateDelay = 75; // Wait at least 75ms before updating neopixel array.
const int NUMPIXELS = 2;


Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, J5X5, NEO_GRB + NEO_KHZ800);
// HSV code inspired by https://en.wikipedia.org/wiki/HSL_and_HSV
// Note, this version is suboptimal to make it easier for me to explain. There
// are better versions out there.
//
uint32_t hsvColor(double h, double s, double v) {  // h is 0.-360., s and v are 0 to 1.0
  // h is Hue, s is Saturation, v is Value.
  double r = 0., g = 0., b = 0.;
  // 360 degree color wheel, fmod() stands for Floating point MODulo. It divides the first
  // number by the second number and returns the remainder, but with the sign the same as
  // the original.

  h = fmod(h , 360.0);
  if ( h < 0.0 ) { //
    h = 360.0 - h;
  }

  double c = v * s;
  double hPrime = fmod(h , 60.0) / 60.0;
  double x = c * hPrime;

  if ( h < 60. ) {
    r = c;
    g = x * c;
  } else if ( h < 120. ) {
    r = c - x * c;
    g = c;
  } else if ( h < 180 ) {
    g = c;
    b = x;
  } else if ( h < 240 ) {
    g = c - x;
    b = c;
  } else if ( h < 300 ) {
    r = x;
    b = c;
  } else {
    r = c;
    b = c - x;
  }
  double m = v - c;
  r = constrain(r + m, 0.0, 1.0);
  g = constrain(g + m, 0.0, 1.0);
  b = constrain(b + m, 0.0, 1.0);
  double brightness = 200.0;
  return pixel.Color(int(r * brightness), int(g * brightness), int(b * brightness));
}
void setupNeoSupp() {
  pixel.begin();
  for (int n = 0; n < NUMPIXELS; n++ ) {
    int r = 50, g = 50, b = 50;
    pixel.setPixelColor(n, pixel.Color(r, g, b));
    pixel.show();
  }
}

const int rxLEDOffset = 0;
extern float colorAngle;
void NeoUpdate() {
  unsigned long now = millis();
  // pixel.show() disables interrupts, so it can only be called when the cortex isn't talking.
  readyToUpdateNeoPixels = false;
  nextNeoUpdate = now + minNeoUpdateDelay;
  //  for (int rxN = 0; rxN < 4; rxN++) {
  //    pixel.setPixelColor(rxN + rxLEDOffset, hsvColor(180.0 + 180.0 * rx[rxN], 1.0, 1.0));
  //  }
  pixel.setPixelColor(0, hsvColor(rangesInMM[3] / 4.0, 1.0, 1.0));
  pixel.setPixelColor(1, hsvColor(colorAngle, 1.0, 1.0));
  pixel.show();
}
void pollNeoSupp() {

  unsigned long now = millis();
  if ( now > nextNeoUpdate ) { // Set a flag

    readyToUpdateNeoPixels = true;
    if ( now > nextNeoUpdate + 1000) {
      NeoUpdate();
    }
  }
}


