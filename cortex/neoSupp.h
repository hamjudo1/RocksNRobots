#include <Adafruit_NeoPixel.h>
extern const long minNeoUpdateDelay; // Wait at least 75ms before updating neopixel array.
extern const int NUMPIXELS;
// J5X1  Arduino9 (up)
extern const int J5X1 ;
extern const char *J5X1name ;
// J5X2  Arduino8 (right)
extern const int J5X2;
extern const char *J5X2name;
// J5X3  (Down)
extern const int J5X3 ;
extern const char *J5X3name ;
// J5X4  (left)
extern const int J5X4 ;
extern const char *J5X4name ;
// j5X5 (front)
extern const int J5X5 ;
extern const char *J5X5name ;

extern uint32_t hsvColor(double h,double s,double v);

extern void setupNeoSupp();
extern void pollNeoSupp();
extern Adafruit_NeoPixel pixel;
extern void NeoUpdate();



