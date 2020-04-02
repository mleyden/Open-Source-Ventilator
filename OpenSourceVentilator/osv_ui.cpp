#include "osv_ui.h"
#include "osv_config.h"
#include "arduino_nano.h"

//#include <stdint.h>
#include <Arduino.h>
#include <Stream.h>   // Wire is an object of type Stream
#include <Wire.h>     // I2C protocol (flawed library)
#include <jm_Wire.h>  // The jm_wire library corrects the longstanding I2C hanging problem with the arduino Wire library.
#include <LiquidCrystal_PCF8574.h> // Matthias Hertel - http://www.mathertel.de/Arduino/LiquidCrystal_PCF8574.aspx 

#ifdef TM1638                      // Keyboard / display / LED combo board
#include <TM1638plus.h>            //  By Gavin Lyons - https://github.com/gavinlyonsrepo/TM1638plus
TM1638plus tm(pin_Strobe_TM, pin_Clock_TM , pin_DIO_TM);  //Constructor object
#endif

void setDisplayMenu(int ph, int dly=std_dly, int param=0) 
{
 dispDelay=dly;   // duration for which the info will be displayed
 dispCnt=0;       // resets the display delay
 dispPhase=ph;    // which page should be displayed
 dispParam=param; // Page dependant additional parameter
 dispTick=0;      // Can be used as a blink counter or similar 
}


/*
 *   displays a set of bars representing the scaled pressure onscreen or with Leds
 */

void displayBargraph (uint16_t value)
{
// add alternate ways of displaying the bargraph here
  
#ifdef TM1638bargraph
  for (uint8_t position = 0; position < 8; position++)
   {
    tm.setLED(position, value & 1);
    value = value >> 1;
   }
#endif  
}

void updateBargraph(boolean mode, long motor_position)
{
 if (mode)  // CV Mode - bargraph needs to show compression
  {
   bargraphDot=4;
   byte b=1*(relPressure<-0.2*compressionScale)+2*(relPressure<-0.1*compressionScale)+4+  // Calculate the bargraph content from the pressure
          8*(relPressure>0.1*compressionScale)+16*(relPressure>0.2*compressionScale)+
          32*(relPressure>0.4*compressionScale)+64*(relPressure>0.8*compressionScale)+
          128*(relPressure>compressionScale);
   barGraph=b; 
  } else
  {           // CP Mode - bargraph needs to show volume 
   bargraphDot=1;
   float vol=motor_position/motorVolumeRatio;
   int c=int ((7*vol)/volumeScale+1); // Always make sure at least the leftmost position is active
   barGraph=0;
   while (c--) barGraph+=barGraph+1;
  }
}

void* initDisplay(void *wire)
{
  #ifdef I2C
  TwoWire *wire_ptr = (TwoWire *)wire;
  #else
  Wire *wire_ptr = (Wire *)wire;
  #endif
  
  #ifdef PCF8574LCDDisplay
  LiquidCrystal_PCF8574 lcd(0x27);   // set the LCD address to 0x27 for a 16 chars and 2 line display
  
  wire_ptr->beginTransmission(0x27);     // I2C Address of the PCF8574 chip 
  int error = wire_ptr->endTransmission();
  if (error == 0) {
    Serial.println("LCD");
    lcd.begin(16, 2); // initialize the lcd
    lcd.setBacklight(255);
    lcd.home();
    lcd.clear();
  } else { 
    Serial.print(error);
  }
  return &lcd;
#endif
}

void doDisplay(void *display)
{
#ifdef PCF8574LCDDisplay

  LiquidCrystal_PCF8574 *lcd_ptr = (LiquidCrystal_PCF8574 *)display;
  
 byte b=0;
 int n=-1;
 while (b<39) if (disp[b++]=='\n')
  {
   disp[b-1]=0;  
   n=b;
  }
 disp[38]=0;   // Safety
 if (n!=1)
  {
    lcd_ptr->clear();
    lcd_ptr->setCursor(1, 0);
    lcd_ptr->print(disp);
  }
 if (n>=0)
  {
   lcd_ptr->setCursor(1,1);
   lcd_ptr->print(&disp[n]);  
  }
#endif
#ifdef TM1638Display
 byte f=0;
 byte c=8;
 while (disp[f]) if (disp[f++]=='.') c++; // Calculates the stringt length without the dot characters
 disp[c]=0; // cut string length 
 tm.displayText(disp);   
 byte l=strlen(disp)-c+8;                 // Fills the right end of the display
 while (l<8) tm.displayASCII(l++,' ');
#endif  
}

#ifdef isKeyboard
/* returns a byte with a bit for each of up to 16 buttons. data is raw, not debounced
 *
 * ading another kind of keyboard should be done here 
 *
 */


uint16_t readKeyboard()  
{
#ifdef TM1638Keyboard
 return tm.readButtons(); // returns a byte with values of button s8s7s6s5s4s3s2s1  
#endif 
#ifdef analogKeyboard
byte b=1;
byte c=btns;
while (c--)
 {
  if ((analogKeys>KbdVals[c]-KbdTols[c]) && (analogKeys<KbdVals[c]+KbdTols[c])) return b;
  b+=b;
 }
return 0; 
#endif 
}

void debounceKeyboard()
{
 uint16_t b = readKeyboard(); // returns a byte with values of button s8s7s6s5s4s3s2s1
 uint16_t c=maxBtn;
 uint16_t m=1;
 while (c--)  //Keyboard debounce
  {
   if (m & b)
     {if (kcnt[c]>debounce) keys[c]=true; else kcnt[c]++;}
    else
     {if (kcnt[c]==0) keys[c]=false; else kcnt[c]--;}
   m+=m; 
  }
}
#endif

#ifdef Beeper
void beep(int lng,byte err)  // Launch a beep sound
{
 if (err) alarm=err; 
 pinMode(pin_Beep, OUTPUT); 
 if (lng) digitalWrite(pin_Beep, LOW);  // active (inverted)
 beepCnt=max(beepCnt,lng);              //  if overlapping sounds, use the longest of them 
}
#endif
