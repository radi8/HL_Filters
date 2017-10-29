// Features are chosen here. Uncomment the desired features

//#define FEATURE_HL1               // HL1 uses data lines for filter switching
//#define FEATURE_HL2               // HL2 uses I2C for filter switchingavrdude -c

//#define FEATURE_Use_Hardware_Pin_for_MOX
#define FEATURE_Quisk
//#define FEATURE_pihpsdr
//#define FEATURE_Custom_Filter
//#define FEATURE_I2C_LCD
//#define FEATURE_SERIAL_PRINT
#define FEATURE_Invert_Inputs     // Allows inverting or non-inverting buffers from HL to Arduino
#define FEATURE_Invert_ptt_line   // Use for open collector driver where ptt tx is low active

// Debug Defines follow here
#define DEBUG_ENABLED            // Must enable this if any debug at all is enabled

// The software can be tested on an Arduino if the commands which affect oscillator pins are ignored
// Rptt = Pin 8 (PB7) and I4   = Pin7 (PB6) go to the crystal and not available on the Arduino board
// As is D0 and D1 which are the serial inputs used for bootloading and serial monitoring
//#define DEBUG_ARDUINO_MODE
#define DEBUG_BAND_CHANGE        // Check the values arriving on User0 to User3
#define DEBUG_PTT_CHANGE
//#define DEBUG_SHOW_FILTER_SWITCH_SIGNALS

#if defined  DEBUG_ENABLED
  #ifndef FEATURE_I2C_LCD
//    #define FEATURE_I2C_LCD
  #endif
#endif

#if defined(FEATURE_I2C_LCD)
  // Setup LCD stuff for 20 col x 4 row display
  #define lcdNumCols 20     // -- number of columns in the LCD
  #define lcdNumRows  4     // -- number of rows in the LCD
  #define lcdAddr  0x27     // -- set the LCD address to suit display
#endif


// The port and pin for each filter are defined as a 16 bit number with the HIGH 8 bits holding the port's pin
// and the LOW 8 bits holding the PORT. they are retrieved by "PIN = v >> 8, and PORT address = v & 0xFF"
// The Hi pass receive filters
const uint16_t HPthru  = (PB2 << 8) + &PORTB; // Pin 14 (PB2) set = Through pass selected, no filters in circuit
const uint16_t HP160   = (PB3 << 8) + &PORTB; // Pin 15 (PB3) set = 160 up selected
const uint16_t HP80    = (PB1 << 8) + &PORTB; // Pin 13 (PB1) set = 80 up selected
const uint16_t HP40    = (PB0 << 8) + &PORTB; // Pin 12 (PB0) set = 40 up selected
const uint16_t HP30    = (PD7 << 8) + &PORTD; // Pin 11 (PD7) set = 30 up selected
const uint16_t HP17    = (PD6 << 8) + &PORTD; // Pin 10 (PD6) set = 17 up selected

// The Lo pass Transmit filters (The 12/10 filter is always in circuit and not switched)
const uint16_t LP160   = (PC3 << 8) + &PORTC;  // pin 26 set = 160M filter selected. Clear = filter not selected
const uint16_t LP80    = (PC2 << 8) + &PORTC;  // pin 25 set = 80M filter selected. Clear = filter not selected
const uint16_t LP60_40 = (PC1 << 8) + &PORTC;  // pin 24 set = 60/40M filter selected. Clear = filter not selected
const uint16_t LP30_20 = (PC0 << 8) + &PORTC;  // pin 23 set = 30/20M filter selected. Clear = filter not selected
const uint16_t LP17_15 = (PB5 << 8) + &PORTB;  // pin 17 set = 17/15M filter selected. Clear = filter not selected
const uint16_t LPthru  = (PB4 << 8) + &PORTB;  // pin 16 set = LP12_10 Roofing filter only, always in cct.

/*
// The following define which ATMega pins activate which TX filter. Note that the 12/10M filter is always
// in line as a roofing filter
const uint8_t LP12_10 = 0;  // All Band bits clear = 10M filter in circuit and all other filters out.
const uint8_t LP160   = A3; // pin 26 (PC3) set = 160M filter selected. Clear = filter not selected
const uint8_t LP80    = A2; // pin 25 (PC2) set = 80M filter selected. Clear = filter not selected
const uint8_t LP60_40 = A1; // pin 24 (PC1) set = 60/40M filter selected. Clear = filter not selected
const uint8_t LP30_20 = A0; // pin 23 (PC0) set = 30/20M filter selected. Clear = filter not selected
const uint8_t LP17_15 = 13; // pin 17 (PB5) set = 17/15M filter selected. Clear = filter not selected
const uint8_t LPthru  = 12; // pin 16 (PB4) set = LP12_10 Roofing filter only, always in cct.

// The following define which ATMega pins activate which RX filter
const uint8_t HPthru  = 10; // Pin 14 (PB2) set = Through pass selected, no filters in circuit
const uint8_t HP160   = 11; // Pin 15 (PB3) set = 17 up selected
const uint8_t HP80    = 9;  // Pin 13 (PB1) set = 17 up selected
const uint8_t HP40    = 8;  // Pin 12 (PB0) set = 17 up selected
const uint8_t HP30    = 7;  // Pin 11 (PD7) set = 17 up selected
const uint8_t HP17    = 6;  // Pin 10 (PD6) set = 17 up selected
*/

// The following defines the pins for the TX and RX path switches. Note either one or the other is set
// but ever both at once for either off or on states.
const uint8_t Tptt  = 5;  // Pin 9 (PD5) Set connects the Tx path from PA out to Antenna
const uint8_t Rptt  = 7;  // Pin 8 (PB7) Set connects the Rx path from Antenna to Rx in via the Tx relay path
const uint8_t mox   = 6;  // Pin 6 (PB6) Receives the MOX value from HL. HIGH = Rx LOW = Tx

// Here we build the map of what filters will be selected from the word formed
// by the bits on USER0,1,2,3 inputs. If 2 filters need to be set, they should
// be or'd together e.g. (LP12_10 || LP17_15)

#if defined(FEATURE_Quisk)

// Here we define the bands selected by the band button clicked in Quisk

//          Button  Filter   Band selected
// ------   ------  ----     --------       
const uint8_t _500K = 0;  // 500 KHz band, Through pass with 10 metre Lo Pass always in circuit
const uint8_t _160M = 1;  // 160 M band,   160M plus LP12_10M filters selected
const uint8_t _80M  = 2;  // 80 M band,    LP80 plus LP12_10M filters selected
const uint8_t _60M  = 4;  // 60 M band,    LP60_40 plus LP12_10M filters selected
const uint8_t _40M  = 5;  // 40 M band,    LP60_40 plus LP12_10M filters selected
const uint8_t _30M  = 6;  // 30 M band,    LP30_20 plus LP12_10M filters selected
const uint8_t _20M  = 7;  // 20 M band,    LP30_20 plus LP12_10M filters selected
const uint8_t _17M  = 8;  // 17 M band,    LP17_15 plus LP12_10M filters selected
const uint8_t _15M  = 9;  // 15 M band,    LP17_15 plus LP12_10M filters selected
const uint8_t _12M  = 0;  // 12 M band,    Through pass with 10 metre Lo Pass always in circuit
const uint8_t _10M  = 0;  // 10 M band,    Through pass with 10 metre Lo Pass always in circuit
const uint8_t _6M   = 0;  // 6  M band,    Thru filter selected

#endif // defined(FEATURE_Quisk)

#if defined(FEATURE_pihpsdr)

#endif

#if defined(FEATURE_Custom_Filter)

#endif

