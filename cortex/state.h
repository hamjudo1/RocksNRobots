extern long longestQuietTime  ;
extern int tookTooLongCount ;
extern boolean readyToUpdateNeoPixels  ;
extern unsigned long nextNeoUpdate ;
extern int stuffQueued;
extern int stuffUnqueued;
extern int SPISlaveInterrupts;
extern int SPISlaveCharactersRead;
extern int SPISlaveCharactersWritten;
extern int packetrx; // count of packets received.
extern boolean radioInitialized;
extern boolean showBrainStemLog;
extern boolean showXn297LLog;
extern boolean showPacketLog; 
extern boolean showRanges;
extern boolean unreadPacket; // We have read a packet from the XN297L, but it hasn't been sent to the brainstem yet.
extern boolean unprocessedPacket;
//Decoded packet state.
#define AUXNUMBER 16
extern int rangesInMM[5];
extern float rx[4];
extern char aux[AUXNUMBER];
extern char lastaux[AUXNUMBER];
extern char auxchange[AUXNUMBER];
extern int rxdata[15];
typedef struct rangeConfigElem {
  int8_t j5Index;
  boolean enabled;
  const char *Name;
  int rangeInMM;
} rangeConfigElem_t;

// The Status byte from the Cortex has these bits:
#define STATE_CORTEX_ALIVE      0x01
#define STATE_BOUND             0x02
#define STATE_GOT_PACKET        0x04
#define STATE_WANT_IMU_DATA     0x08
#define STATE_WANT_BATTERY_DATA 0x10
#define STATE_MUST_BE_ZERO      0x80 // If this isn't zero, the SPI bus is stuck high.
     // If the bus is briefly stuck high, that means the cortex CPU is rebooting or
     // a test probe is touching multiple pins.
     // If this is permanent you have proof that the hardware was hand assembled
     // with care, but not precision.
     // 
extern int cortexState;
// When connecting test equipment to the SPI bus,
// it is easy to accidentally pull the bus high. When that
// happens, every bit is stuck at 1. That might be temporary
// or permanent, but we shouldn't do anything dangerous.
extern boolean stateIndicateHardwareFault; 

enum cortex_debug {
  packet_in_fifo=0,
  we_are_bound,
  radio_confused,
  range_display,
  cortex_loop,
};
//typedef enum cortex_debug corex_debug_t;
extern void cortexDebugRange(enum cortex_debug  debug_event, int rangeFinderIndex, int rangeInMM);
extern void cortexDebug(enum cortex_debug debug_event);
extern void pollDebug();
extern void setupDebug();
extern void quietTime();
extern void dispXn297LLog();
extern void dispPacketLog();
extern void pollRangeFinders();
extern void setupRangeFinders();
extern void pollBlink();
extern void blinkIt(int pinNo);
extern void replaceRx(float newrx[4], int oldPacket[15], int newPacket[15] ); 
extern void updateChecksum(int packet[15]);

