// Features are chosen here. Uncomment the desired features

//#define FEATURE_HL1               // HL1 uses data lines for filter switching
#define FEATURE_HL2               // HL2 uses I2C for filter switching
//#define FEATURE_Quisk
//#define FEATURE_pihpsdr
//#define FEATURE_Custom_Filter
#define FEATURE_Invert_Inputs     // Allows inverting or non-inverting buffers from HL to Arduino
#define FEATURE_Invert_ptt_line   // Use for open collector driver where ptt tx is low active

// Debug Defines follow here
#define DEBUG_ENABLED            // Must enable this if any debug at all is enabled
#define DEBUG_BAND_CHANGE        // Check the values arriving on User0 to User3
#define DEBUG_PTT_CHANGE
#define DEBUG_SHOW_FILTER_SWITCH_SIGNALS

#if defined  DEBUG_ENABLED
  #ifndef FEATURE_I2C_LCD
    #define FEATURE_I2C_LCD
  #endif
#endif

#if defined(FEATURE_I2C_LCD)
  // Setup LCD stuff for 20 col x 4 row display
  #define lcdNumCols 20     // -- number of columns in the LCD
  #define lcdNumRows  4     // -- number of rows in the LCD
  #define lcdAddr  0x27     // -- set the LCD address to suit display
#endif


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

// The following defines the pins for the TX and RX path switches. Note either one or the other is set
// but ever both at once for either off or on states.
const uint8_t Tptt  = 5;  // Pin 9 (PD5) Set connects the Tx path from PA out to Antenna
const uint8_t Rptt  = 8;  // Pin 8 (PB7) Set connects the Rx path from Antenna to Rx in via the Tx relay path


// Here we build the map of what filters will be selected from the word formed
// by the bits on USER0,1,2,3 inputs. If 2 filters need to be set, they should
// be or'd together e.g. (LP12_10 || LP17_15)

#if defined(FEATURE_Quisk)

// Here we map the button clicked to the band relay
const uint8_t txFilterMap[16] = {
// Filter   Button  Band          Selected
// ------   ------  ----          --------  
  LP12_10,  // 0,   No band,      10 metre Lo Pass only filter selected
  LP160,    // 1,   500 KHz band, thru filter selected
  LP160,    // 2,   160 M band,   LP160 plus LP12_10M filters selected
  LP80,     // 3,   80 M band,    LP80 plus LP12_10M filters selected
  LP60_40,  // 4,   60 M band,    LP60_40 plus LP12_10M filters selected
  LP60_40,  // 5,   40 M band,    LP60_40 plus LP12_10M filters selected
  LP30_20,  // 6,   30 M band,    LP30_20 plus LP12_10M filters selected
  LP30_20,  // 7,   20 M band,    LP30_20 plus LP12_10M filters selected
  LP17_15,  // 8,   17 M band,    LP17_15 plus LP12_10M filters selected
  LP17_15,  // 9,   15 M band,    LP17_15 plus LP12_10M filters selected
  LP12_10,  // 10,  12 M band,    10 metre Lo Pass only filter selected
  LP12_10,  // 11,  10 M band,    10 metre Lo Pass only filter selected
  LPthru,   // 12,   6 M band,    thru filter selected
  LPthru,   // 13,  Through pass. i.e. no filters with thru connection
  LPthru,   // 14,  Through pass. i.e. no filters with thru connection
  LPthru    // 15,  Through pass. i.e. no filters with thru connection
};

const uint8_t rxFilterMap[16] = {
// Filter   Button  Band          Selected
// ------   ------  ----          --------  
  HP160,    // 0,   No band,      HP160 only filter selected
  HPthru,   // 1,   137 KHz band, thru filter selected
  HP160,    // 2,   160 M band,   HP160 only filter selected
  HP80_60,  // 3,   80 M band,    HP160 and HP80_60 filters selected
  HP80_60,  // 4,   60 M band,    HP160 and HP80_60 filters selected
  HP40_30,  // 5,   40 M band,    HP160 and HP40_30 filters selected
  HP40_30,  // 6,   30 M band,    HP160 and HP40_30 filters selected
  HP20_17,  // 7,   20 M band,    HP160 and HP20_17 filters selected
  HP20_17,  // 8,   17 M band,    HP160 and HP20_17 filters selected
  HP15_10,  // 9,   15 M band,    HP160 and HP15_10 filters selected
  HP15_10,  // 10,  12 M band,    HP160 and HP15_10 filters selected
  HP15_10,  // 11,  10 M band,    HP160 and HP15_10 filters selected
  HP15_10,  // 12,   6 M band,    HP160 and HP15_10 filters selected
  HPthru,   // 13,  Through pass. i.e. no filters with thru connection
  HPthru,   // 14,  Through pass. i.e. no filters with thru connection
  HPthru    // 15,  Through pass. i.e. no filters with thru connection
};
#endif

#if defined(FEATURE_pihpsdr)

#endif

#if defined(FEATURE_Custom_Filter)

#endif

