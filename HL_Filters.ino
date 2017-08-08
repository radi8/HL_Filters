/*****************************************************************
 * 
 * Copyright ©2015 Graeme Jury ZL2APV
 * Released under the GPL v2 License - Please alter and share.
 * Control lines expansion for Hermes-Lite

 * Filename: HL_Filters.ino
 *
 * This program runs on an Atmega328P using the internal clock.
 * 
The Hermes-Lite project was first pioneered by Steve Haynal KF7O and the first set of boards and practical
build was pioneered by Rob Frohne KL7NA. From there numerous others have joined in to create this game
changing project.
*****************************************************************/

/*
                               +-----+
                  +------------| USB |------------+
                  |            +-----+            |
LP17_15      B5   | [ ]D13/SCK        MISO/D12[ ] |   B4 LPthru
                  | [ ]3.3V           MOSI/D11[ ]~|   B3 HP160
                  | [ ]V.ref     ___    SS/D10[ ]~|   B2 HPthru
LP30_20      C0   | [ ]A0       / N \       D9[ ]~|   B1 HP80
LP60_40      C1   | [ ]A1      /  A  \      D8[ ] |   B0 HP40
LP80         C2   | [ ]A2      \  N  /      D7[ ] |   D7 HP30
LP160        C3   | [ ]A3       \_0_/       D6[ ]~|   D6 HP17
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



////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main code starts here
////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Add includes here
#include "Features_and_Settings.h"

#if defined(FEATURE_I2C_LCD)    // A 3.3 volt I2C LCD display is used to print debug messages and status
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
#endif
// End of includes

// Global values
#if defined(FEATURE_I2C_LCD)
  // Set the pins on the I2C chip used for LCD connections:
  //                       addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
  LiquidCrystal_I2C lcd(lcdAddr, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#endif

void setup() {
  // Setup the ports
  DDRB = 0b10111111;  // bits 0..5 outputs, bit 6 input, bit 7 output.
  DDRC = 0b00001111;  // bits 0 .. 3 outputs, bits 4 .. 7 inputs.
  DDRD = 0b11100000;  // bits 0 .. 4 inputs, bits 5 .. 7 outputs.
//Set up so 160M Hi pass and 10M Lo pass filters in circuit with filters in RX mode
  PORTB = 0b11001000;
  PORTC = 0b10000000;
  PORTD = 0b00000000;
  
#if defined(FEATURE_I2C_LCD)
  lcd.begin(lcdNumRows, lcdNumCols);
#endif  

}

void loop() {
  // put your main code here, to run repeatedly:

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutines start here
////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(FEATURE_I2C_LCD)
void lcdPrintSplash()
{
  lcd.home();                   // go home
  lcd.print(F("ARDUINO TUNER by"));
  lcd.setCursor (0, 1);        // go to the next line
  lcd.print(F("ZL2APV (c) 2015 "));
  //  lcd.backlight(); // finish with backlight on
  //  delay ( 5000 );
}
#endif
/**********************************************************************************************************/
