/*****************************************************************

   Copyright ©2015 Graeme Jury ZL2APV
   Released under the GPL v2 License - Please alter and share.
   Control lines expansion for Hermes-Lite

   Filename: HL_Filters.ino

   This program runs on an Atmega328P using the internal clock.

  The Hermes-Lite project was first pioneered by Steve Haynal KF7O and the first set of boards and practical
  build was pioneered by Rob Frohne KL7NA. From there numerous others have joined in to create this game
  changing project.
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
      6 I4      in    |       6 RESET   in    |       6 HP17  Pullup on PB6 and PB7 output Hi sets Rx modeout
      7 Rptt    out   |       7 ???     in    |       7 HP30  out

      These are the crystal pins and normally set with PORTB, Pullup on PB6 and PB7 output Hi sets Rx mode6 = Hi and PORTB, 7 = L0 with no pullups.
    ** Serial in pins needed for bootloading, PORTD, 0 = RXD and PORTD, 1 = TXD with no pullups
*/

//const uint16_t LP12_10 = (PORTC << 8) + PC3;  // All Band bits clear = 10M filter in circuit and all other filters out.

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main code starts here
////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Add includes here
#include "Features_and_Settings.h"
#include <Wire.h>
#if defined(FEATURE_I2C_LCD)    // A 3.3 volt I2C LCD display is used to print debug messages and status
#include <LiquidCrystal_I2C.h>
#endif
// End of includes

// I2C Globals and constants
#define baudRate 115200
const uint8_t MY_ADDRESS = 42;
#ifdef FEATURE_I2C_LCD
const uint8_t OTHER_ADDRESS = lcdAddr;
#endif
char I2C_sendBuf[32];
uint8_t I2C_recBuf[32];
uint8_t FILT = 0; //Commands received are placed in this variable

boolean last_state = HIGH;

struct status {
  uint8_t J16signals;
  uint16_t txFilterNum;
  uint16_t rxFilterNum;
  boolean MOX_State; // true = Rx, false = Tx
}
_status;

status lastState = _status;



// System global values
const boolean RX = true;
const boolean TX = false;

#if defined(FEATURE_I2C_LCD)
// Set the pins on the I2C chip used for LCD connections:
//                       addr, en,rw,rs,d4,d5,d6,d7,bl,blpolthe same
LiquidCrystal_I2C lcd(lcdAddr, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#endif

void setup() {
  // Setup the port directions
#ifdef DEBUG_ARDUINO_MODE // We lose D0 and D1 plus B6 and B7 in Arduino mode
  DDRB = DDRB | B00111111; // Don't change its 6,7 as these are oscillator
#else
  DDRB = 0b10111111;  // bits 0..5 outputs, bit 6 input, bit 7 output.the same
#endif
  DDRC = 0b00001111;  // bits 0 .. 3 outputs, bits 4 .. 7 inputs.
#ifdef DEBUG_ARDUINO_MODE
  DDRD = DDRD | B11111100; // Don't change bits 0, 1 as these are serial pins
  DDRD = DDRD & B11100011;
#else
  DDRD = 0b11100000;  // bits 0 .. 4 inputs, bits 5 .. 7 outputs.
#endif

  //Set up port states for outputs and pullups for inputs
#ifdef DEBUG_ARDUINO_MODE
  PORTB = PORTB & 0b11000000; // Clear everything except oscillator settings
  PORTB = PORTB | 0b00011000; // Turn on HP160 and LPthru filters
#else
  PORTB = 0b11011000; // Pullup on PB6 with PB7, 3, 2 outputs Hi sets Rx mode with HP160 and LPthru
#endif
  PORTC = 0b10110000; // SCL & SDA Pullups
  PORTD = 0b00011111; // Pullups for inputs PD0..4

  // Set the board's initial Status to match port settings
  _status.J16signals = 0x07; // Assume not connected initially
  _status.txFilterNum = LPthru;
  _status.rxFilterNum = HP160;
#ifdef DEBUG_ARDUINO_MODE
  _status.MOX_State = TX; // We can't set the  RX mode as its pin (PB, 7) is the oscillator
#else
  _status.MOX_State = RX; // true = Rx, false = Tx
#endif
  Wire.begin (MY_ADDRESS);
  Wire.onReceive (receiveEvent);
#if defined(FEATURE_I2C_LCD) // Setup the display if enabled
  lcd.begin(lcdNumRows, lcdNumCols);
  lcd_PrintSplash();
#endif

#ifdef FEATURE_SERIAL_PRINT
  //Initialize serial and wait for port to open:
  Serial.begin(baudRate);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  PrintSplash();
#endif
  lastState = _status;
  clearFilters();
}

void loop() {
#if defined(FEATURE_I2C_LCD)
  lcd_DisplayStatus();
#endif

  // Poll the input pins for a filter or ptt change and set the _status struct accordingly. The
  // struct will be checked against the last time the pins were polled and processed if changed.
#ifndef DEBUG_ARDUINO_MODE  // This is an oscillator pin in Arduino mode so don't read
  _status.MOX_State = (PINB & (1 << mox));
#endif
  _status.J16signals = 0;
  _status.J16signals = ((PIND & 0b00011100) >> 2);

  // Special case when J16 = 0xFF (nothing connected, so all pins pulled high)
  // or J16 = 0x00 (no filter selected so use default).
  if (_status.J16signals != lastState.J16signals) { // Only process input pins if something changed
    Serial.print(F("Changed input = ")); Serial.println(_status.J16signals, BIN);
    switch (_status.J16signals & 0b00000111)
    {
      case 1: // 160 Metre band.
        _status.txFilterNum = LP160;
        _status.rxFilterNum = HP160;
        break;
      case 2: // 80 Metre band.
        _status.txFilterNum = LP80;
        _status.rxFilterNum = HP80;
        break;
      case 3: // 60/30 Metre band.
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
        // LP 30 MHz will be set in clearFilters() subroutine
        _status.rxFilterNum = HP17;
        break;
      default:
        _status.txFilterNum = LPthru;
        _status.rxFilterNum = HPthru;
    }
    applyStatus();
    PrintSplash();
  }

#ifndef DEBUG_ARDUINO_MODE
  if ((_status.txFilterNum != lastState.txFilterNum) | (_status.rxFilterNum != lastState.rxFilterNum) |
      (_status.MOX_State != lastState.MOX_State)) {
    applyStatus();
#ifdef FEATURE_SERIAL_PRINT
    PrintSplash();
#endif
  }


#else
  if ((_status.txFilterNum != lastState.txFilterNum) | (_status.rxFilterNum != lastState.rxFilterNum)) {
    applyStatus();
#ifdef FEATURE_SERIAL_PRINT
    PrintSplash();
#endif
  }
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutines start here
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
  uint16_t *port;
  uint16_t pin;

  Serial.println(F("Hermes-Lite Smart Filter v1.0.0"));
  Serial.print(F("_status.J16signals  = ")); Serial.println(_status.J16signals, BIN);
  Serial.print(F("_status.txFilterNum = ")); Serial.println(_status.txFilterNum, HEX);
  Serial.print(F("_status.rxFilterNum = ")); Serial.println(_status.rxFilterNum, HEX);
  Serial.print(F("_status.MOX_State   = ")); Serial.println(_status.MOX_State);

  // Print the port and pin for the TX filters
  Serial.print(F("HP Filter = "));
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
  Serial.print(F("LP Filter = "));
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
  uint16_t *port;
  uint16_t pin;

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
  uint16_t *port;
  uint16_t pin;

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
  *port = (_status.txFilterNum & 0x00FF);
  pin = (_status.txFilterNum >> 8);
  *port |= (1 << pin);

  // Turn on the new RX filter
  *port = (_status.rxFilterNum & 0x00FF);
  pin = (_status.rxFilterNum >> 8);
  *port |= (1 << pin);
  lastState = _status;
}

void clearFilters() // Does not change PORTB 6,7 (osc) or PORTD 0,1 (serial)
{
  // Clear all the HP and LP filters except leave the LPthru filter connected
  PORTB |= (1 << PB4); // Set the LPthru filter
  PORTB &= 0b11010000; // PORTB 5, 3..0 = LP17_15, HP160, HPthru, HP80, HP40 cleared
  PORTC &= 0b11110000; // PORTC 3..0 = LP160, LP80, LP60-40, LP30-20 cleared
  PORTD &= 0b00111111; // PORTC 7,6 = HP30, HP17 cleared
}
/************************** I2C subroutines ***************************************************************/

// The slave is listening for filter switching commands from the master. Embedded into the filter
// values is the ptt state which is held in bit 7. Bits 6..4 contain the selected Tx filter value.
// Bit 4 is for use and bits 3..0 hold the Rx filter value. For any change of state e.g band change,
// ptt action including a CW key press or frequency excursion beyond the band edge this 8 bit value
// is sent. It is the only data sent from Hermes-Lite so any received will be a valid ptt or filter.

// The receiveEvent captures the sent command in the filt variable and updates the _status struct
// with ptt state, Tx filter value and Rx filter value.

void receiveEvent(int howMany)
// called by I2C interrupt service routine when any incoming data arrives.
// The command is sent as a uint8_t
{
  uint8_t filt = Wire.read();

  lastState = _status;
  // Get the ptt state as we use this to decode the received FILT byte as a HPF or LPF value
#if defined(FEATURE_Use_Hardware_Pin_for_MOX)
  // I4 (PORTB, 6) can be used for MOX or else the high order bit of I2C data
  _status.MOX_State = PINB, mox;
#else
  _status.MOX_State = (filt & 0b10000000); // High order bit = MOX
#endif
  if (_status.MOX_State) { // We are receiving an Rx filter value
    _status.rxFilterNum = (filt & 0b00000111);
  } else { // We are receiving a Tx filter value
    _status.txFilterNum = ((filt & 0b01110000) >> 4);
  }
#if defined(DEBUG_SHOW_FILTER_SWITCH_SIGNALS)
  lcd.home();
  lcd.print("@receiveEvent()");
  lcd.setCursor (0, 1);
  lcd.print("filt = ");
  lcd.print(filt);
#endif
}

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
                Serial.println(I2C_sendBuf); */
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

  */
  Wire.write(I2C_sendBuf);
  //  Serial.println(I2C_sendBuf); // debug
}  // end of sendSensor

/**********************************************************************************************************/
