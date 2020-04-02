#ifndef OSV_UI_H
#define OSV_UI_H

#include "osv_config.h"

//#include <stdint.h>
#include <Arduino.h>

#define std_dly 70  // display standard delay

#ifdef TM1638Keyboard
#define maxBtn  8   // should be 8 or 16
#define btnPrev 7   // position for the 'Prev'  button
#define btnNext 6   // position for the 'Next'  button
#define btnDn   5   // position for the 'Down'  button
#define btnUp   4   // position for the 'Up'    button
#define btnOk   0   // position for the 'Enter' button
#define debounce 3  // Keyboard debouncing
#endif

#ifdef analogKeyboard
#define maxBtn  8   // should be 8 or 16
#define btnPrev 3   // position for the 'Prev'  button
#define btnNext 4   // position for the 'Next'  button
#define btnDn   6   // position for the 'Down'  button
#define btnUp   5   // position for the 'Up'    button
#define btnOk   7   // position for the 'Enter' button
#define debounce 3  // Keyboard debouncing
const int KbdVals[] = {335,510,765,934,1022};
const int KbdTols[] = {17,26,39,47,52};
#define btns    5
#define analogTol 0.05 // percentage of allowed error
#endif

#ifdef TM1638Display
//#define dispBufferLength 12
#endif

#ifdef PCF8574LCDDisplay
#define dispBufferLength 40
#endif

#ifdef dispMenus     // Alternate displays must also enable this
char    disp[dispBufferLength];// Display data buffer
#endif

#ifdef isKeyboard        
byte    keys[maxBtn],          // Debounced keyboard keys
        kcnt[maxBtn],          // Keyboard debounce counter
        mkeys[maxBtn];         // Key memory (allows to detect button transitions)
#ifdef analogKeyboard
int    analogKeys;            // Buffered analog value for keyboard
#endif
#endif

int dispDelay,      // delay before the next display update or change
    dispPhase,      // Main variable that controls the display and menu start machine. Contains the current screen number
    dispCnt,        // Counter for dispDelay
    beepCnt,               // Counter for the delay before the end of the current beep (or 0). Time is in 10ms increments
    dispParam;      // additional parameter used on certain display screens
byte dispTick,      // Auxilary value for the dislpay. Can be used as a blink counter or similar
     barGraph,      // segments of the pressure bar graph
     bargraphDot,   // rate indicator blinking segment on the bargraph
     alarm;         // 0 normal, else alarm number
     
float relPressure,  // relative pressure (measured pressure - ambient pressure)
      compressionScale,      // Scale of the compression relative to set value
      volumeScale;           // Scale of the volume relative to set value
      
void setDisplayMenu(int ph, int dly, int param);
void displayBargraph (uint16_t value);
void updateBargraph(boolean mode, long motor_position);

void* initDisplay(void *wire);
void doDisplay(void *disp);

uint16_t readKeyboard();
void debounceKeyboard();

void beep(int lng,byte err);

#endif /* !OSV_UI_H */
