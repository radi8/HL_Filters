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
                  | [ ]A6              INT1/D3[ ]~|   D3 I2 (Input with pullups)
                  | [ ]A7              INT0/D2[ ] |   D2 I1 (Input with pullups)
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


  PortB 0 HP40    out   | PortC 0 LP30_20 out   | PortD 0 NC    in (pullup)
      1 HP80    out   |       1 LP60_40 out   |       1 NC    in (pullup)
      2 HPthru  out   |       2 LP80    out   |       2 I1    in
      3 HP160   out   |       3 LP160   out   |       3 I2    in
      4 LPthru  out   |       4 SDA     in    |       4 I3    in
      5 LP17_15 out   |       5 SCL     in    |       5 Tptt  out
      6 I4      in    |       6 RESET   in    |       6 HP17  out
      7 Rptt    out   |       7 ???     in    |       7 HP30  out
*/

////////////////////////////////const uint16_t LP12_10 = (PORTC << 8) + PC3;  // All Band bits clear = 10M filter in circuit and all other filters out.//////////////////////////////////////////////////////////////////////////////
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
const uint8_t MY_ADDRESS = 42;
const uint8_t OTHER_ADDRESS = lcdAddr;
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
  // Setup the ports
  DDRB = 0b10111111;  // bits 0..5 outputs, bit 6 input, bit 7 output.the same
  DDRC = 0b00001111;  // bits 0 .. 3 outputs, bits 4 .. 7 inputs.
  DDRD = 0b11100000;  // bits 0 .. 4 inputs, bits 5 .. 7 outputs.
  //Set up so 160M Hi pass and 10M Lo pass filters in circuit with filters in RX mode
  PORTB = 0b11011000; // Pullup on PB6 and PB7 sets Rx mode
  PORTC = 0b10110000; // SCL & SDA Pullups
  PORTD = 0b00011111; // Pullups for inputs PD0..4

  // Set the board's initial Status
  _status.J16signals = 0xFF; // Assume not connected initially
  _status.txFilterNum = LPthru;
  _status.rxFilterNum = HP160;
  _status.MOX_State = RX;

  Wire.begin (MY_ADDRESS);
  Wire.onReceive (receiveEvent);
#if defined(FEATURE_I2C_LCD) // Setup the display if enabled
  lcd.begin(lcdNumRows, lcdNumCols);
  lcd_PrintSplash();
#endif

}

void loop() {
#if defined(FEATURE_I2C_LCD)
  lcd_DisplayStatus();
#endif

  // Poll the input pins for a filter or ptt change
  _status.J16signals = ((PINB, mox) << 3);
  _status.J16signals |= (((PIND, 4) << 2) + ((PIND, 3) << 1) + (PIND, 2));

  // Special case when J16 = 0xFF (nothing connected, so all pins pulled high)
  // or J16 = 0x00 (no filter selected so use default).
  if (_status.J16signals != 0xFF) { // Only process input pins if something is connected
    if (_status.J16signals == 0x00) { // Switch to default filter case
      _status.txFilterNum = LPthru;
      _status.rxFilterNum = HP160;
      _status.MOX_State = ((_status.J16signals & 0b00001000) >> 3);
    } else { // Calculate which filters to connect
      switch (_status.J16signals & 0b00000111)
      {
        case 1: // 160 Metre band.
          clearFilters();
          digitalWrite(LP160, HIGH);
          digitalWrite(HP160, HIGH);                    
          break;
        case 2: // 80 Metre band.
          clearFilters();
          digitalWrite(LP80, HIGH);
          digitalWrite(HP80, HIGH);
          break;
        case 3: // 60/30 Metre band.
          clearFilters();
          digitalWrite(LP60_40, HIGH);
          digitalWrite(HP40, HIGH);        
          break;
        case 4: // 30/20 Metre band.
          clearFilters();
          digitalWrite(LP30_20, HIGH);
          digitalWrite(HP30, HIGH);
          break;
        case 5: // 18/15 Metre band.
          clearFilters();
          digitalWrite(LP17_15, HIGH);
          digitalWrite(HP17, HIGH);
          break;
        case 6: // 12/10 Metre band.
          clearFilters();
          // LP 30 MHz has been set in clearFilters() subroutine
          digitalWrite(HP17, HIGH);
          break;
        default:
          // Turn all the filters off
          digitalWrite(LP160, LOW);
          digitalWrite(LP80, LOW);
      }
    }
  }

  if ((_status.txFilterNum != lastState.txFilterNum) | (_status.rxFilterNum != lastState.rxFilterNum) |
      (_status.MOX_State != lastState.MOX_State)) {
    applyStatus();
  }
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
#if defined(FEATURE_I2C_LCD)
void lcd_DisplayStatus()
{
  // Print the _status struct on a 2 line 16 column display
  lcd.home();
  //lcd.print(F("0123456789012345")); I am using this for display layout template
  lcd.print(F("HP = "));
  lcd.print(_status.txFilterNum); // Tx filters = 1 to 6.
  lcd.print(F(" & LP = "));
  lcd.print(_status.rxFilterNum); // Rx filters = 1 to 5.
  lcd.setCursor (0, 1);        // go to the next line (column, row)
  lcd.print(F("TR = "));
  if(_status.MOX_State){
    lcd.print("Rx"); // T-R_State true = "Rx"
  } else {
    lcd.print("Tx"); // T-R_State false = "Tx"
  }
  lcd.print(F(", Dat = "));
  lcd.print(_status.J16signals);
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
    PORTB |= (1 << Rptt);   // Set to Rx mode
  } else {
    PORTB &= ~(1 << Rptt);  // Clear Rx mode
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
}

void clearFilters()
{
  // Clear all the HP and LP filters except leave the LPthru filter connected
  PORTB |= (1 << PB4); // Set the LPthru filter
  PORTB &= 0b11010000; // PORTB 5, 3..0 = LP17_15, HP160, HPthru, HP80, HP40 cleared
  PORTC &= 0b11110000; // PORTC 3..0 = LP160, LP80, LP60-40, LP30-20 cleared
  PORTD &= 0b00111111; // PORTC 7,6 = HP30, HP17 cleared
}
/************************** I2C subroutines ***************************************************************/

// The slave is listening for filter switching commands from the master. Embedded into the filter
// values is the ptt state which is held in bit 7. Bits 0..6 contain the selected filter value.
// The the filter value and ptt state is sent is sent for any band change, ptt action including a
// CW key press or frequency excursion beyond the band edge
// The receiveEvent captures the sent command in the filt variable and updates the _status struct
// with ptt state and filter value, setting the changed flag if a change occurred.

void receiveEvent(int howMany)
// called by I2C interrupt service routine when any incoming data arrives.
// The command is sent as a uint8_t
{
  uint8_t filt = Wire.read();
  Serial.print("@Slave:receiveEvent(), FILT = ");
  Serial.println(filt);

  lastState = _status;
  // Get the ptt state as we use this to decode the received FILT byte as a HPF or LPF value
#if defined(FEATURE_Use_Hardware_Pin_for_MOX)
  // I4 (PORTB, 6) is used for MOX or the high order bit of I2C data
  _status.MOX_State = PINB, mox;
#else
  _status.MOX_State = (filt & 0b10000000); // High order bit = MOX
#endif
  if (_status.MOX_State) { // We are receiving an Rx filter value
    _status.rxFilterNum = (filt & 0b00000111);
  } else { // We are receiving a Tx filter value
    _status.txFilterNum = ((filt & 0b01110000) >> 4);
  }
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
