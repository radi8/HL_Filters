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
