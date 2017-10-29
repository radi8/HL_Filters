/*****************************************************************

   Copyright ©2015 Graeme Jury ZL2APV
   Released under the GPL v2 License - Please alter and share.
   Control lines expansion for Hermes-Lite

   Filename: HL_Filters.ino

   This program runs on an Atmega328P using the internal clock.

  The Hermes-Lite project was first pioneered by Steve Haynal KF7O and the first set of boards and
  practical build was pioneered by Rob Frohne KL7NA. From there numerous others have joined in to
  create this game changing project.
*****************************************************************/

/*
                               +-----+
                  +------------| USB |------------+
                  |            +-----+            |
  LP17_15    B5   | [ ]D13/SCK        MISO/D12[ ] |   B4 LPthru
                  | [ ]3.3V           MOSI/D11[ ]~|   B3 HP160
                  | [ ]V.ref     ___    SS/D10[ ]~|   B2 HPthru
  LP30_20    C0   | [ ]A0       / N \       D9[ ]~|   B1 HP80
  LP60_40    C1   | [ ]A1      /  A  \      D8[ ] |   B0 HP40
  LP80       C2   | [ ]A2      \  N  /      D7[ ] |   D7 HP30
  LP160      C3   | [ ]A3       \_0_/       D6[ ]~|   D6 HP17
             C4   | [ ]A4/SDA               D5[ ]~|   D5 Tptt
             C5   | [ ]A5/SCL               D4[ ] |   D4 I3 (Input with pullups)
  n/c     Pin19   | [ ]A6              INT1/D3[ ]~|   D3 I2 (Input with pullups)
  n/c     Pin22   | [ ]A7              INT0/D2[ ] |   D2 I1 (Input with pullups)
                  | [ ]5V                  GND[ ] |
             C6   | [ ]RST                 RST[ ] |   C6 RESET
                  | [ ]GND   5V MOSI GND   TX1[ ] |   D0
                  | [ ]Vin   [ ] [ ] [ ]   RX1[ ] |   D1
                  |          [ ] [ ] [ ]          |
                  |          MISO SCK RST         |
                  | NANO-V3                       |
                  +-------------------------------+

  Rptt = Pin 8 (PB7) Wires to the crystal in an Arduino and N/A. Using it as a data line in this cct.
  I4   = Pin7 (PB6) Crystal input N/A on Arduino. (Input with pullups) on J6 connector
                  http://busyducks.com/ascii-art-arduinos


  PortB 0 HP40    out   | PortC 0 LP30_20 out   | PortD 0 NC    in (Serial) **
        1 HP80    out   |       1 LP60_40 out   |       1 NC    in (Serial) **
        2 HPthru  out   |       2 LP80    out   |       2 I1    in
        3 HP160   out   |       3 LP160   out   |       3 I2    in
        4 LPthru  out   |       4 SDA     in    |       4 I3    in
        5 LP17_15 out   |       5 SCL     in    |       5 Tptt  out
        6 I4      in    |       6 RESET   in    |       6 HP17  out
        7 Rptt    out   |       7 ???     in    |       7 HP30  out

       Pullup on PB6 forces PB7 output Hi and sets Rx mode
       These are the crystal pins and normally set with PORTB, Pullup on PB6 and PB7 output Hi sets Rx
       mode6 = Hi and PORTB, 7 = L0 with no pullups.
    ** Serial in pins needed for bootloading, PORTD, 0 = RXD and PORTD, 1 = TXD with no pullups
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                          Code starts here
////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Add includes here
#include "Features_and_Settings.h"
#include <Wire.h>
#if defined(FEATURE_I2C_LCD)    // A 3.3 volt I2C LCD display is used to print debug messages and status
#include <LiquidCrystal_I2C.h>
#endif
// End of includes

//Serial and I2C Globals and constants
#define baudRate 19200
const uint8_t MY_ADDRESS = 0x20; // Fixed by Hermes-Lite requirement
#ifdef FEATURE_I2C_LCD
const uint8_t OTHER_ADDRESS = lcdAddr;
#endif

//char I2C_sendBuf[32];
//uint8_t I2C_recBuf[32];
// uint8_t FILT = 0; //Commands received are placed in this variable

//boolean last_state = HIGH;

struct status {
  uint8_t J16signals;
  uint16_t txFilterNum;
  uint16_t rxFilterNum;
  boolean MOX_State; // true = Rx, false = Tx
  boolean auxValue; // true = switch on 160M Rx filter, false filter switched off
} _status;

status lastState = _status;



// System global values
const boolean RX = true;
const boolean TX = false;
boolean gHL1mode = false;

#if defined(FEATURE_I2C_LCD)
// Set the pins on the I2C chip used for LCD connections:
//                       addr, en,rw,rs,d4,d5,d6,d7,bl,blpolthe same
LiquidCrystal_I2C lcd(lcdAddr, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#endif

void setup() {
  // Setup the port directions
#ifdef DEBUG_ARDUINO_MODE // We lose D0 and D1 plus B6 and B7 in Arduino mode
  DDRB = DDRB | B00111111; // Don't change its 7,6 as these are oscillator
  DDRD = DDRD | B11100000; // Don't change bits 1, 0 as these are serial pins. Set bits 7..5
  DDRD = DDRD & B11100011; // Make sure bits 4..2 are inputs (D4, D3, D2 on Arduino)
#else
  DDRB = 0b10111111;  // bit 7 output, bit 6 input, bits 5 .. 0 outputs.
  DDRD = 0b11100000;  // bits 7 .. 5 outputs, bits 4 .. 0 inputs.
#endif
  DDRC = 0b00001111;  // bits 7 .. 4 inputs.bits 3 .. 0 outputs.

  //Set up port states for outputs and pullups for inputs
#ifdef DEBUG_ARDUINO_MODE
  PORTB = PORTB & 0b11000000; // Clear everything except oscillator settings
  PORTB = PORTB | 0b00011000; // Turn on HP160 and LPthru filters
  PORTD |= 0b00011100; // Pullups for inputs PD2..4
#else
  PORTB = 0b11011000; // Pullup on PB6 with PB7, 3, 2 outputs Hi sets Rx mode with HP160 and LPthru
  PORTD = 0b00011111; // Pullups for inputs PD0..4
#endif
  PORTC = 0b10110000; // SCL & SDA Pullups

  // Set the board's initial Status to match port settings
  _status.J16signals = 0x07; // Assume not connected initially
  _status.txFilterNum = LPthru;
  _status.rxFilterNum = HP160;
#ifdef DEBUG_ARDUINO_MODE
  _status.MOX_State = TX; // We can't set the  RX mode as its pin (PB, 7) is the oscillator
#else
  _status.MOX_State = RX; // true = Rx, false = Tx
#endif

  // Setup the I2C
  Wire.begin (MY_ADDRESS);
  Wire.onReceive (receiveEvent);
#if defined(FEATURE_I2C_LCD) // Setup the display if enabled
  lcd.begin(lcdNumRows, lcdNumCols);
  lcd_PrintSplash();
#endif

#ifdef FEATURE_SERIAL_PRINT
  //Initialize serial and wait for port to open:
  Serial.begin(baudRate);
  //  while (!Serial) {
  //    ; // wait for serial port to connect. Needed for Leonardo only
  //  }
  PrintSplash();
#endif
  lastState = _status;
  applyStatus(); // Set filters to initial state in case the board is not connected to a radio.
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Main Loop starts here
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

#if defined(FEATURE_I2C_LCD)
  lcd_DisplayStatus();
#endif

  // Check to see whether an I2C device is connected or if the shorting plug is plugged in on the
  // I2C connector. If the shorting plug is installed (HL1 mode), poll the input pins for a filter
  // or pttchange and set the _status struct accordingly. The struct will be checked against the
  // last time the pins were polled and processed if changed. I2C operates via an interrupt and
  // does not need to be polled.
  if ((PINC & 0b000110000) == 0) { // Test for SDA and SCL both LOW (possibly shorting plug in)
    delay(1);
    if ((PINC & 0b000110000) == 0) { // After short delay test again for SDA and SCL both LOW
      gHL1mode = true;              // and if so we must have the shorting plug plugged in.
    }
  }
  else {
    gHL1mode = false;
  }

  if (gHL1mode) { // Only process if we are using a Hermes-Lite version HL1
    /*
        // In case there is RF from Hermes-Lite we switch to the transmit filters in order to
        // isolate the receiver input. It is OK to hot switch the Tx filters.
      #ifndef DEBUG_ARDUINO_MODE  // Don't change this in Arduino mode as it is the oscillator pin
        PORTB &= ~(1 << Rptt);  // Clear Rx mode
      #endif
        PORTD |= (1 << Tptt);   // Set to Tx mode


          digitalWrite(Tptt, HIGH);
          delay(1000);
          digitalWrite(Tptt, LOW);
          delay(1000);
          digitalWrite(Tptt, HIGH);
          delay(1000);
          digitalWrite(Tptt, LOW);
          delay(1000);
    */

    // Now get the state of the J16 lines from HL1
//    _status.J16signals = 0; //TODO check if this is an overkill and remove if so
    _status.J16signals = ((PIND & 0b00011100) >> 2);

    if ((_status.J16signals != lastState.J16signals)) { // Only process input pins if filter changed.
#ifdef FEATURE_SERIAL_PRINT
      Serial.print(F("Changed input = ")); Serial.println(_status.J16signals, BIN);
#endif
      switch (_status.J16signals)
      {
        case 1: // 160 Metre band.
          _status.txFilterNum = LP160;
          _status.rxFilterNum = HP160;
          break;
        case 2: // 80 Metre band.
          _status.txFilterNum = LP80;
          _status.rxFilterNum = HP80;
          break;
        case 3: // 60/40 Metre band.
          _status.txFilterNum = LP60_40;
          _status.rxFilterNum = HP40;
          break;
        case 4: // 30/20 Metre band.
          _status.txFilterNum = LP30_20;
          _status.rxFilterNum = HP30;
          break;
        case 5: // 17/15 Metre band.
          _status.txFilterNum = LP17_15;
          _status.rxFilterNum = HP17;
          break;
        case 6: // 12/10 Metre band.
          _status.txFilterNum = LPthru;
          _status.rxFilterNum = HP17;
          break;
        case 7: // Roof and Floor filters.
          _status.txFilterNum = LPthru;
          _status.rxFilterNum = HP160;
          break;
        default:
          _status.txFilterNum = LPthru;
          _status.rxFilterNum = HPthru;
      }
#ifdef FEATURE_SERIAL_PRINT
      Serial.println(F("Processed a filter change"));
#endif
    }

    // Having processed the filter commands, we now get the actual state of MOX .
#ifndef DEBUG_ARDUINO_MODE
    _status.MOX_State = (PINB & (1 << mox)); // This is an oscillator pin in Arduino mode so
    // don't attempt to read read on an Arduino.
#endif
  } // Finished reading and recording state of input pins
  
  if ((_status.txFilterNum != lastState.txFilterNum) | (_status.rxFilterNum != lastState.rxFilterNum) |
      (_status.MOX_State != lastState.MOX_State)) {
    applyStatus();
#ifdef FEATURE_SERIAL_PRINT
    PrintSplash();
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Subroutines start here
////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(FEATURE_I2C_LCD)
void lcd_PrintSplash()
{
  lcd.home();                   // go home
  lcd.print(F("Hermes-Lite     "));
  lcd.setCursor (0, 1);        // go to the next line
  lcd.print(F("Smart Filter v1 "));
}
#endif

/**********************************************************************************************************/
#ifdef FEATURE_SERIAL_PRINT
void PrintSplash()
{
  uint8_t *port;
  uint8_t pin;

  Serial.println(F("Hermes-Lite Smart Filter v1.0.0"));
  Serial.print(F("_status.J16signals  = ")); Serial.println(_status.J16signals, BIN);
  Serial.print(F("_status.txFilterNum = ")); Serial.println(_status.txFilterNum, HEX);
  Serial.print(F("_status.rxFilterNum = ")); Serial.println(_status.rxFilterNum, HEX);
  Serial.print(F("_status.MOX_State   = ")); Serial.println(_status.MOX_State);

  // Print the port and pin for the TX filters
  Serial.print(F("LP Filter = "));
  *port = (_status.txFilterNum & 0x00FF);
  pin = (_status.txFilterNum >> 8); // Shift hi order 8 bits to lo order position and drop lo order
  if (*port == &PORTB) {
    Serial.print(F("PORTB, "));
  } else {
    Serial.print(F("PORTC, "));
  }
  Serial.println(pin);

  // Print the port and pin for the RX filters
  *port = (_status.rxFilterNum & 0x00FF);
  pin = (_status.rxFilterNum >> 8); // Shift hi order 8 bits to lo order position and drop lo order
  Serial.print(F("HP Filter = "));
  if (*port == &PORTB) {
    Serial.print(F("PORTB, "));
  } else {
    Serial.print(F("PORTD, "));
  }
  Serial.println(pin);
}
#endif

/**********************************************************************************************************/
#if defined(FEATURE_I2C_LCD)
void lcd_DisplayStatus()
{
  uint8_t *port;
  uint8_t pin;

  // Print the _status struct on a 2 line 16 column display
  lcd.home();
  //lcd.print(F("0123456789012345")); I am using this for display layout template
  //lcd.print(F("HP = B5: LP = D6"));

  // Print the port and pin for the TX filters
  lcd.print(F("HP = "));
  *port = (_status.txFilterNum & 0x00FF);
  pin = (_status.txFilterNum >> 8); // Shift hi order 8 bits to lo order position and drop lo order
  if (*port == &PORTB) {
    lcd.print("B");
  } else {
    lcd.print("C");
  }
  lcd.print(pin);

  // Print the port and pin for the RX filters
  *port = (_status.rxFilterNum & 0x00FF);
  pin = (_status.rxFilterNum >> 8); // Shift hi order 8 bits to lo order position and drop lo order
  lcd.print(F(": LP = "));
  if (*port == &PORTB) {
    lcd.print("B");
  } else {
    lcd.print("D");
  }
  lcd.print(pin);

  // Print the ptt state and the J16 pins value as Hex. (Need to decode manually)
  lcd.setCursor (0, 1);        // go to the next line (column, row)
  // 0123456789012345
  // TR=Rx:J16=0xFF
  lcd.print(F("TR = "));
  if (_status.MOX_State) {
    lcd.print("Rx"); // T-R_State true = "Rx"
  } else {
    lcd.print("Tx"); // T-R_State false = "Tx"
  }
  lcd.print(F(":J16="));
  lcd.print(_status.J16signals, HEX);
}
#endif

/**********************************************************************************************************/
void applyStatus()
{
  uint8_t pin;

  // First set transmit/receive state regardless of previous state i.e. may duplicate current setting.
  if (_status.MOX_State == RX) {
    PORTD &= ~(1 << Tptt);  // Clear Tx mode
#ifndef DEBUG_ARDUINO_MODE  // Don't change this in Arduino mode as it is the oscillator pin
    PORTB |= (1 << Rptt);   // Set to Rx mode
#endif
  } else {
#ifndef DEBUG_ARDUINO_MODE  // Don't change this in Arduino mode as it is the oscillator pin    
    PORTB &= ~(1 << Rptt);  // Clear Rx mode
#endif
    PORTD |= (1 << Tptt);   // Set to Tptt mode
  }

  clearFilters();

  // Turn on the new TX filter
  pin = (_status.txFilterNum >> 8);

#define T_PORT (* (volatile uint8_t *) (_status.txFilterNum & 0x00FF))
  T_PORT |= (1 << pin);

  // Turn on the RX filter only if in RX mode otherwise leave cleared for better Rx to Tx isolation
  if (_status.MOX_State == RX) {
    pin = (_status.rxFilterNum >> 8);
#define R_PORT (* (volatile uint8_t *) (_status.rxFilterNum & 0x00FF))
    R_PORT |= (1 << pin);
    
    // Here we check to see if Aux value is set and switch in the 160M filter if so
    if(_status.auxValue) {
      pin = (HP160 >> 8);
#define A_PORT (* (volatile uint8_t *) (HP160 & 0x00FF))
        A_PORT |= (1 << pin);     
    }
  }

  lastState = _status;
}

/**********************************************************************************************************/
void clearFilters() // Does not change PORTB 6,7 (osc) or PORTD 0,1 (serial)
{
  // Clear all the HP and LP filters
  PORTB &= 0b11000000; // PORTB 5..0 = LP17_15, LPthru, HP160, HPthru, HP80, HP40 cleared
  PORTC &= 0b11110000; // PORTC 3..0 = LP160, LP80, LP60-40, LP30-20 cleared
  PORTD &= 0b00111111; // PORTD 7,6 = HP30, HP17 cleared
}

/************************** I2C subroutines ***************************************************************/

// The slave is listening for filter switching commands from the master. Embedded into the filter
// values is the ptt state which is held in bit 7. Bits 6..4 contain the selected Tx filter value.
// Bit 3 is for general use. Bits 2..0 hold the Rx filter value. For any change of state e.g band
// change, ptt action including a CW key press or frequency excursion beyond the band edge this 8 bit
// value is sent. It is the only data sent from Hermes-Lite so any received will be a valid ptt or filter.

// The receiveEvent captures the sent command in the filt variable and updates the _status struct
// with ptt state, Tx filter value and Rx filter value. Polling in main loop detects change to _status.

void receiveEvent(int howMany)
// called by I2C interrupt service routine when any incoming data arrives.
// The data is only ever sent as a single uint8_t so there is no need for byte count checking.
{
  uint8_t filt = Wire.read();
  uint8_t rxValue;
  uint8_t txValue;
//  uint8_t auxValue;

  // Get the ptt state
  _status.MOX_State = (filt & 0b10000000); // High order bit = MOX (LOW = Tx, HIGH = Rx)

  //Extract the transmit, receive filter values and auxilliary bit value.
  txValue = ((filt & 0b01110000) >> 4);
  rxValue = (filt & 0b00000111);
  _status.auxValue = ((filt & 0b00001000) >> 3); // We use this to decide to turn on 160M filter or not

  // Build the value to go into the _status.txFilterNum
  switch (txValue)
  {
    case 1: // 160 Metre band.
      _status.txFilterNum = LP160;
      break;
    case 2: // 80 Metre band.
      _status.txFilterNum = LP80;
      break;
    case 3: // 60/30 Metre band.
      _status.txFilterNum = LP60_40;
      break;
    case 4: // 30/20 Metre band.
      _status.txFilterNum = LP30_20;
      break;
    case 5: // 17/15 Metre band.
      _status.txFilterNum = LP17_15;
      break;
    case 6: // 12/10 Metre band.
      _status.txFilterNum = LPthru;
      break;
    case 7: // Roof and Floor filters.
      _status.txFilterNum = LPthru;
      break;
    default:
      _status.txFilterNum = LPthru;
  }

  // Build the value to go into the _status.rxFilterNum
  switch (rxValue)
  {
    case 1: // 160 Metre band.
      _status.rxFilterNum = HP160;
      break;
    case 2: // 80 Metre band.
      _status.rxFilterNum = HP80;
      break;
    case 3: // 60/30 Metre band.
      _status.rxFilterNum = HP40;
      break;
    case 4: // 30/20 Metre band.
      _status.rxFilterNum = HP30;
      break;
    case 5: // 17/15 Metre band.
      _status.rxFilterNum = HP17;
      break;
    case 6: // 12/10 Metre band.
      _status.rxFilterNum = HP17;
      break;
    case 7: // Roof and Floor filters.
      _status.rxFilterNum = HP160;
      break;
    default:
      _status.rxFilterNum = HPthru;
  }
#ifdef FEATURE_SERIAL_PRINT  
  Serial.println(F("Signal processed via I2C"));
#endif  
#if defined(DEBUG_SHOW_FILTER_SWITCH_SIGNALS)
  lcd.home();
  lcd.print("@receiveEvent()");
  lcd.setCursor (0, 1);
  lcd.print("filt = ");
  lcd.print(filt);
#endif
}

/**********************************************************************************************************/
/*
  void requestEvent()
  // This is called if the master has asked the slave for information. The
  // command to identify which info has been received by receiveEvent and
  // placed into the global "FILT" variable.
  {
  //  Serial.print("@Slave:requestEvent(), FILT = ");
  //  Serial.println(FILT, 10);
  switch (FILT)
  {
      //    case FILT_READ_A0: sendSensor(A0, _volts); break;  // send A0 value
      //    case FILT_READ_A1: sendSensor(A1, _amps); break;  // send A1 value
      //    case FILT_READ_A2: sendSensor(A2, _analog2); break;  // send A2 value
      //    case FILT_READ_D2: Wire.write(digitalRead(2)); break;   // send D2 value
      //    case FILT_READ_D3: Wire.write(digitalRead(3)); break;   // send D3 value
      //    case FILT_STATUS: sendStatus();
      //    case FILT_ID: {
      //        memset(I2C_sendBuf, '\0', 32); // Clear the I2C Send Buffer
      //        strcpy(I2C_sendBuf, "Slave address = 9");
      /*
          int len = strlen(I2C_sendBuf);
          I2C_sendBuf[len] = '\0';
          for (byte i = 0; i <= len; i++) {
            Wire.write(I2C_sendBuf[i]); // Chug out 1 character at a time
          }  // end of for loop
          /*    Wire.write(I2C_sendBuf);
                Serial.print("@Slave:requestEvent(), Response sent = ");
                Serial.println(I2C_sendBuf); star, slash
      //          Wire.write(I2C_sendBuf);
      //        break;   // send our ID
      //      }
  }  // end of switch
  FILT = 0;
  }

  void sendSensor (const byte which, uint8_t cmd)
  // The integer value of the analog port is converted to a string and sent.
  {
  int val = analogRead (which);
  //  uint8_t len;

  memset(I2C_sendBuf, '\0', 32); // Clear the I2C Send Buffer
  sprintf(I2C_sendBuf, "%d %d", cmd, val);
  //  len = strlen(I2C_sendBuf);
  /*
    I2C_sendBuf[len] = '\0';
    for (byte i = 0; i <= len; i++)
    {
      Wire.write(I2C_sendBuf[i]); // Chug out one char at a time.
    }  // end of for loop

  star, slash
  Wire.write(I2C_sendBuf);
  //  Serial.println(I2C_sendBuf); // debug
  }  // end of sendSensor
*/
/**********************************************************************************************************/
