/*

  Open source Ventilator


    0.10 Initial public version

    0.11 Add interactive terminal command line complete control (through USB link) 
         A minimal device without screen or keyboard is possible with the same software.

    0.12 Add "Assist Control" mode with adaptive negative pressure trigger cycle. 
         This mode was requested by a MD : "ER doc here chiming in, hopefully 
         I can answers some questions or clarify minimal requirements.The basic 
         mode you need is Assist Control. This mode provides a breath whenever 
         the patient triggers a breath (by inhaling and creating negative pressure 
         past a certain threshold) or whenever the patient is due for a breath 
         (based on the set rate). The breath it provides are either a fixed volume 
         (and the doc monitors the pressures delivered) or a fixed pressure (and 
         the doc monitors the volume delivered). AC/volume is often called Volume 
         Control, and AC/pressure is often called Pressure Control. While I do not 
         have direct experience ventilating COVID patients yet, these 2 modes will 
         cover 99% of ICU cases." 
         This version also adds editing, permanent storage and USB setting of the new 
         Patient Synchronization parameter and an audible alarm when pressure remains 
         above 120% of preset pressure for 1 second or mode.

    0.13 This version is reorganized for better code modularity, asymmetric breathing
         and bug fix. It is a major reorganization of the code done to get a more 
         modular software, ready for bringing in other sensors, a different user 
         interface and ports to other controller architectures. Very long functions 
         are broken down into smaller functionnal blocks that should be much easier 
         to understand and modify, each global variable is commented. 
         This version also implements most of the guidelines in the UK minimal emergency 
         ventilator device specifications. Namely, all variable limits have been tweaked 
         and the variable Inspiratory:Expiratory ratio is fully functionnal. 
         A bug that allowed floating point variables retrieved from a garbage filled EEPROM 
         to bypass the checks leading to malfunction have been corrected (NaN detection).

    0.14 This version include proper timing, speed and acceleration for the motor control.
         The pressure controlled and volume controlled modes are implemented.

    0.15 Minor version change.
         This version allows the replacement of the buggy Wire.h arduino library  (can hang
         the controller) with acorrect version known as jm_Wire
         https://github.com/ermtl/Open-Source-Ventilator/blob/master/OpenSourceVentilator/README.md
         'Wire.h' is still the default library to prevent compiler errors.
         The processor's hardware watchdog can now be enabled (off by default, use with care, you
         risk bricking your processor. 
         Modularisation is getting better (work in progress)

    0.16 Double pressure sensor, breath phases modularization
         This version can use 2 absolute sensors (or still use just 1, configurable) to
         constantly monitor the ambient air pressure. This is important for patients that 
         are heliported and for patients placed in a negative pressure room, 2 situations
         where the ambient pressure can change rapidly.
         The 2 sensors together will behave as a differential pressure sensor.
         Since pressure is a critical data, several safeguards and coping strategies have 
         been added to make sure data from the sensor is accurate and there is a failsafe.
         Things such as randomly connecting/disconnecting sensors are non blocking and behave 
         as expected.
         this feature requires the #define jm_Wire to be uncommented.Please see the link in
         the description for version 0.15 above

         Also, as requested, the various phases and functions managing the breathing cycle 
         itself have been modularized to allow third parties to implement more sophisticated
         control strategies.

         Finally, the processor's program memory is filing up quickly as the control becomes 
         more complex. There is still opportunities for important optimizations, but the base 
         version will have to give up some non essential functions to fit in a regular Arduino
         Nano (such as the verbose serial interface or the ability to manage both the native
         keyboard / display and the USB serial command line interface.

         This version barely fits with the USB commands on, so they've been deactivated.
         You can get them back by uncommenting the #define USBcontrol line.

         The following versions will both target the Nano and the Mega with Atmega2560 processor
         for 8 times the available program memory.

    0.17 2x16 LCD Display - Telemetry
         
         The ability to use an alphanumeric 2 lines * 16 characters LCD display had been
         requested, it will now be fully supported. Since this LCD uses lots of IO pins, 
         the retained version is connected to a PCF8574 "Backpack" board to make it I2C 
         compatible. The LCD+the backpack are sold online for around 2.50$.
         The TM1638 display is still supported as a minimal option.
         A good side effect is that the library is lean and so USB/serial terminal control
         is back by default.
         At this point, the keyboard and bargraph are still done with the TM1638, but a
         simple button keyboard will be supported very soon.
         Telemetry will send information in real time about the breathing cycle for further 
         data analysis and / or graphical display !
         This will be very very useful for people developing mechanical solutions, and 
         doing data analysis to develop motor position <=> volume mapping
         Data is sent in common CSV format, around 1 data point every 20 millisecond
         The "T" command, sent through the serial port toggles telemetry on or off.

         every data point includes :
         - Time since the breathing was activated.
         - Breathing cycle phase (more phases might be added in the future)
           0 : Start of new cycle
           1 : Inspiration
           2 : Start of expiration
           3 : Expiration
           4 : Patient breathing initiation search
           5 : Patient initiated a breathing cycle
          - Motor position (in step) Travelled distance depends on driver microstepping
                                     and driving mechanism.
          - relative pressure in cm H2O
          In addition to these data points, during phase 0 (start of cycle), the following 
          cycle data is sent:
          - Ambient pressure (mBar)
          - Breathing volume per cycle in ml
          - Motor destination / target (steps)
          - Breathing length (in seconds)
          - Breathing in duration (in seconds)
          - breathing in top speed (in steps/second)
          - Motor acceleration during inspiration (in steps per second squared)
          - Breathing out duration (in seconds)
          - breathing out top speed (in steps/second)
          - Motor acceleration during expiration (in steps per second squared)

          
    0.20 Realtime graphics, Keyboard, trajectory
         
         This version adds realtime graphics, compatible with the Arduino serial plotter.
         This is an extension of the telemetry in Version 0.17, completed and scaled properly
         for display.
         It shows 2 series of values. The first is updated in realtime (every 20ms) and shows 
         the breathing cycle itself, the commanded motor position, it's actual movement, the 
         pressure sensor output and alarm number.
         The second set is updated once per cycle and shows various values linked to timing, 
         speed, acceleration for various parts of the cycle.
         This data should be extremely useful for developping mechanical prototypes and shows
         the ability to use a remote HMI if desired.
         define ArduinoPlotter for proper visualisation scaling, comment it for real telemetry data 
         This version also add a keyboard as an alternative to the TM1638 keyboard.
         The keyboard used is made with 5 buttons all connected to a single analog input for
         simpler cabling and preserving controller pins.
         https://electronics.stackexchange.com/questions/101409/how-to-debouce-six-buttons-on-one-analog-pin-with-arduino
         The keyboard I use have the following values:
         - 10K to ground
         - 'Enter' button connected to +5V
         - 'Prev'  button connected with a 22Ko resistor
         - 'Next'  button connected with a 10Ko resistor
         - 'Up'    button connected with a 3.6Ko resistor
         - 'Down'  button connected with a 1Ko resistor
         The resulting analog values are stored in the KbdVals[] array, and value tolerance in KbdTols[] array
         If you use different resistor values, uncomment debugAnalogKeyboard to get the analog values to fill the 
         array values.
         Please note that when plotting the graph, while the controller would accept serial data input,the arduino plotter 
         does not allow it, so data needs to be changed using the physical keyboard.
         A 3D printed faceplate containing the display, the buttons and the buzzer is available (including FreeCAD drawing)
         in the 3D-Print directory .
         While graphing the data, a few corrections were made to the trajectory engine. 

         

    GPL V3 Licence
    --------------
             
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

  Author: Eric Vinter.
  Created March 19, 2020
  EMAIL : erv at mailpeers dot net
  
*/


#include "osv_config.h"

//********************************   CONNECTION PINS   ********************************

#include "arduino_nano.h"
//#include "arduino_due.h"

//*******************************   REQUIRED LIBRARIES   *******************************

#include "osv_libraries.h"

//***************************************   END   ***************************************

#include "osv_ui.h"


#ifdef BoschBMxSensor
// Connecting the BME280 Sensor:
// Sensor              ->  Board
// -----------------------------
// Vin (Voltage In)    ->  3.3V
// Gnd (Ground)        ->  Gnd
// SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
// SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
#endif


#ifdef BoschBMP180Sensor
#define BMP180_I2C_ADDRESS 0x77
 BMP180I2C bmp180(BMP180_I2C_ADDRESS); //create an BMP180 object using the I2C interface
#endif


#ifdef StepGen
// Define the stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, pin_Stepper_Step, pin_Stepper_DIR);
#endif

#ifdef jm_Wire
extern uint16_t twi_readFrom_timeout; // Allows to add a longer timeout for I2C reads
extern uint16_t twi_writeTo_timeout;  // Allows to add a longer timeout for I2C reads
extern bool twi_readFrom_wait;        // Must be set to true to activate the timeouts
extern bool twi_writeTo_wait;         // Must be set to true to activate the timeouts
#endif

#define SERIAL_BAUD 115200  // Serial port communication speed

#ifdef E2PROM
#define eeStart  48         // EEPROM Offset for data
#endif 

#ifdef CurrentSense
 #define current0          512        // Nominal zero point of the current sensor is VCC/2
 #define currentMaxIdle     50                  // Analog maximum deviation when the motor is idle (including sensor midpoint error)
 #define analogCurrentRatio  0.07398       // Value in Amp per analog step
 #define idleCurrentFilter   0.002  // IIR filtering ratio (lower value produce longer time constant)
 #define currentFilter       0.05   // IIR filtering ratio (lower value produce longer time constant)
#endif

#ifdef disableMotorctrl
 #define motorDriverRecovery 20  // Allow some time (in miliseconds) for the motor driver to recover after being idle
#endif

// #define sweep

//#define serialVerbose   // Should the messages sent to the serial port be verbose or not

#define samplePeriod 20  // 20 ms sensor sampling period
#define highPressureAlarmDetect 10  // delay before an overpressure alarm is triggered (in samplePeriod increments)
#define delayAmbientPressure 2500  // In dual pressure sensor mode, delay when the main ambient pressure sensor fails
                                   // before we start hunting for ambient pressure value on the second sensor  

#define startup_message       0    // power up start
#define error_message        10    // 10 + error code
#define Menu_message        100    // screen for each menu item 
#define Measurement_Start   101    // Screen with "start" message
#define Stop_message        102    // stop indicator screen  
#define Measurement_Message 103    // first of measurement messages

#define MenuItems         7
#define keyIdleReturn 16000    // returns to main menu after 16 seconds without key press
#define motorOffDelay 10000    // Motor is cut off after that time being inactive.


boolean term,                  // true if the terminal is in human mode, false in machine mmode (not fully implemented)
        stopMove;              // Immediately stop motor movements (E-Stop)
int     timCnt,                // Timer interrupt cycle counter used to spread the load over time
        breathGraphCnt,        // Counter for the visual blinking of the bargraph at each breathing cycle start
        timerLvl,              // Count timer interrupt nesting and skips interface update when more than one
        syncDelay,             // Time (in ms) when the patient can symchronize the breathing cycle by inhaling (negative pressure)
        highPressureAlarmCnt;  // delay when the pressure is too high (>1.2x preset value) before an alarm is triggered
boolean doDisp,                // indicates if a display update is required or not.
        sensHumidity,          // True if a humidity sensor is present, false otherwise
        sensTemperature,       // True if a temperature sensor is present, false otherwise
        sensPressure,          // True if a pressure sensor is present, false otherwise
        sensAmbientPressure,   // True if a separate ambient pressure sensor is present, false otherwise
        sensFlow,              // True if an airflow sensor is present, false otherwise (not used yet)
        allSet,                // True if all actual values match the programed values, false when ramping values up or down
        active,                // True if the machine is currently doing breathing cycles, false otherwise
        Mactive,               // active memory (transition detection  
        motorState,            // True if the motor is activated, false otherwise, false otherwise
        CVmode,                // CV or CP mode indicator; 
#ifdef  watchdogProtect        
        watchdogStarted,       // True when the watchdog has been activated. the first few seconds after reset are without watchdog protection
#endif        
        BMP180Phase,           // Indicates if the expected value is a temperature or a pressure
        I2CBusy,               // Tells whenever the sensors are used in the main lop so that the display can wait.
        telemetry,             // true if the serial telemetry mode is active, false otherwise.
        debug;                 // True if a debugging mode is used, false otherwise
        
float   ambientPressure,       // Calculated ambient air pressure (averaged)
        pressure,              // Current pressure, as read from the sensor
        peakPressure,          // high pass filtered value of the pressure. Used to detect patient initiated breathing cycles
        avgPressure,           // Averaged pressure (used to limit the motor speed with short spikes filtered out
        presM1,presM2,         // Memoies used to extrapolate pressure values when a new sensor measurement is not available
        temperature,           // temperature as read by the sensor, in Celsius
        temperature2,          // temperature as read by the 2nd sensor, in Celsius (if available)
        humidity,              // humidity as read by the sensor, in % RH
        breathInSpeed,         // Speed of the breath inhalation in liter / second
        breathInAcceleration,  // Acceleration in step squared/second      
        breathOutSpeed,        // speed of the breath exhalation in liter / second
        breathOutAcceleration; // Acceleration in step squared/second      
long    tick,                  // counter used to trigger sensor measurements
        breathe,               // counter used to track the current breathing cycle
        lastKey,               // last time when a key was pressed (used to automatically return to the main menu) 
        commTime,              // Last time a serial communication was received
        lastActive,            // Last time the motor was active
        startActive,           // last time the motor was started
        lastAmbientPressure,   // last time a valid ambient pressure was collected
        breathLength,          // duration of the current breathing cycle in milliseconds. This is 60000/BPM.
        breathTime,            // Length of cycle minus the sync period
        breathIn,              // Length of the breath inhalation
        breathOut,             // Length of the breath exhalation
        motorTarget,           // Exprected motor position at the end of the cycle
        PIP,                   // Peak pressure
        PEEP;                  // End of cycle pressure 
float   BPM,                   // beats Per Minute. This is the minimum breathing rate, as defined by the user
        volume,                // Breathing volume per cycle in ml
        compression;           // Max / target compression while inhaling
byte    sc,                    // counter for displaying debugging info about the sensors 
        breathPhase,           // Current phase of the breathing cycle;
        menuItem;              // Menu item currently being set by the user
        
        
// Parameters saved to / recovered from EEPROM

float   reqBPM;                // respiratory frequency
float   reqVolume;             // respiratory volume in milliliters 
float   reqCompression;        // compression for the ambu-bag in Pa
float   syncRatio;             // portion of the cycle for breath synchronisation
float   expirationRatio;       // The proportion of each breathing cycle that is spent breathing in compared to breathing out

#ifdef  CurrentSense
int     currentVal;            // ADC result of the current measurement
boolean sensCurrent;           // True if a current sensor is present, false otherwise
float   idleCurrent,           // Averaged value of the ADC when the motor is idle
        currentRaw,            // unfiltered, scaled current value 
        current;               // filtered, scaled current value 
#endif
 

void (* resetFunction)(void) = 0;  // Self reset (to be used with watchdog)

void  __attribute__ ((noinline)) tab ()
{
 if (term) Serial.print('\t');
}

void  __attribute__ ((noinline)) sep ()
{
 if (term) Serial.print(',');
}


boolean checkValues()
{
 boolean isOk=(reqBPM>=minBPM);                      // BPM in allowed range ?
 if (reqBPM>maxBPM) isOk=false;                 
 if (reqVolume<minVolume) isOk=false;                // Volume in allowed range ?
 if (reqVolume>maxVolume) isOk=false;           
 if (reqCompression<minCompression) isOk=false;      // Compression in allowed range ?
 if (reqCompression>maxCompression) isOk=false; 
 if (syncRatio<minSyncRatio) isOk=false;             // portion of the cycle for breath synchronisation in allowed range ?
 if (syncRatio>maxSyncRatio) isOk=false;
 if (expirationRatio<minExpirationRatio) isOk=false; // portion of the cycle for breath synchronisation in allowed range ?
 if (expirationRatio>maxExpirationRatio) isOk=false;
 if (isnan(reqBPM)) isOk=false;                      // Check for malformed floating point values (NaN)
 if (isnan(reqVolume)) isOk=false;  
 if (isnan(reqCompression)) isOk=false;  
 if (isnan(syncRatio)) isOk=false;  
 if (isnan(expirationRatio)) isOk=false;  
 return isOk;     
}

void meas()
{
#ifdef serialVerbose 
 Serial.print(F("Temp:")); 
 Serial.print(temperature);
 Serial.print(F("°C\tHumidity:")); 
 Serial.print(humidity);
 Serial.print(F("% RH\tPressure:")); 
 Serial.print(pressure);
 Serial.print(F(" Pa\tAmbient Pressure:")); 
 Serial.print(ambientPressure);
 Serial.print(F(" Pa\tRelative Pressure:")); 
 Serial.print(relPressure);
 Serial.println(F(" Pa"));  
#else
 Serial.print(temperature);
 tab();
 Serial.print(humidity);
 tab();
 Serial.print(pressure);
 tab();
 Serial.print(ambientPressure);
 tab();
 Serial.println(relPressure);
#endif 
}

typedef struct
  {
   float reqBPM,reqVolume,reqCompression,syncRatio,expirationRatio;
   boolean cvMode,active;
   int ambientPressure;
  } EEPROM_Data;

#ifdef E2PROM
void eeput (int n)  // records to EEPROM (only if values are validated)
{
 int eeAddress = eeStart;
 boolean isOk=checkValues();
    
 if (n==1) isOk=true;  // override (for debug testing)
 if (isOk)
  {
   EEPROM.put(eeAddress, reqBPM);
   eeAddress += sizeof(float);
   EEPROM.put(eeAddress, reqVolume);
   eeAddress += sizeof(float);
   EEPROM.put(eeAddress, reqCompression);
   eeAddress += sizeof(float);
   byte b=(active)?1:0;
   EEPROM.put(eeAddress, b);
   eeAddress += sizeof(byte);
   EEPROM.put(eeAddress, syncRatio);
   eeAddress += sizeof(float);
   EEPROM.put(eeAddress, expirationRatio);
   eeAddress += sizeof(float);
   b=(CVmode)?1:0;
   EEPROM.put(eeAddress, b);
   eeAddress += sizeof(byte);
  }
}    
#endif

void eeget ()
{    
#ifdef E2PROM
 int eeAddress = eeStart;
 byte tac,cvm;

 EEPROM.get(eeAddress, reqBPM);
 eeAddress += sizeof(float);
 EEPROM.get(eeAddress, reqVolume);
 eeAddress += sizeof(float);
 EEPROM.get(eeAddress, reqCompression);
 eeAddress += sizeof(float);
 EEPROM.get(eeAddress, tac);
 eeAddress += sizeof(byte);
 EEPROM.get(eeAddress, syncRatio);
 eeAddress += sizeof(float);
 EEPROM.get(eeAddress, expirationRatio);
 eeAddress += sizeof(float);
 EEPROM.get(eeAddress, cvm);
 eeAddress += sizeof(byte);
 // Check if the data is coherent
 boolean isOk=checkValues();
 if (tac>1) isOk=false; // is the value 0 or 1 ?
 if (cvm>1) isOk=false; // is the value 0 or 1 ?
 if (!isOk) 
  {
   reqBPM          = defaultBPM;
   reqVolume       = defaultVolume;
   reqCompression  = defaultCompression;
   syncRatio       = defaultSyncRatio;
   expirationRatio = defaultExpirationRatio;
   tac=0;
   cvm=1;
  }
 active=(tac>0);  
 CVmode=(cvm>0);  
#ifdef serialVerbose 
 if (term) Serial.println((isOk)?F("Read EEPROM Settings"):F("Read Default Settings"));
#else
 if (term) Serial.println((isOk)?F("ROM"):F("Default"));
#endif 
#else
 reqBPM          = defaultBPM;
 reqVolume       = defaultVolume;
 reqCompression  = defaultCompression;
 syncRatio       = defaultSyncRatio;
 expirationRatio = defaultExpirationRatio;
 active=false;
 CVmode=true;
#ifdef serialVerbose 
 if (term) Serial.print(F("Read Default Settings\n"));
#endif 
#endif
}   


#ifdef USBcontrol

#define CommandMaxLength 30

byte BufferLength;
String Buffer = "*******************************";  // filler

void prstat()
{
#ifdef serialVerbose
 Serial.print  (F("\nPatient assisted breathing\tO"));
 Serial.println((active)?F("N"):F("ff"));
 Serial.print  ((CVmode)?F("CV"):F("CP"));
 Serial.print  (F(" mode\nVentilation speed\t\t"));
 Serial.print  (reqBPM);
 Serial.print  (F(" cycles / minute\n"
                  "Ventilation volume\t\t"));
 Serial.print  (reqVolume/1000);
 Serial.print  (F(" liter\n"
                  "Ventilation pressure\t\t"));
 Serial.print  (reqCompression/1000);
 Serial.print  (F(" KPa\n"
                  "Patient synchonization\t\t"));
 Serial.print  (int(syncRatio*100.1));
 Serial.print  (F(" % of cycle\n"
                "Expiration ratio\t\t"));
 Serial.print  (expirationRatio);
 Serial.println(F(" x inspiration\n"));
#else
 Serial.print ((active)?F("ON\t"):F("Off\t"));
 Serial.print  ((CVmode)?F("CV\t"):F("CP\t"));
 Serial.print  (reqBPM);
 tab();
 Serial.print  (reqVolume/1000);
 tab();
 Serial.print  (reqCompression/1000);
 tab();
 Serial.print  (int(syncRatio*100.1));
 tab();
 Serial.println  (expirationRatio);
#endif 
}

void SerialCommand()
{
  commTime=millis(); 
  float r;
  boolean ok=true;
  char ch=Buffer[1];
  char c=Buffer[0];
  Buffer[0]=32;
  r=Buffer.toFloat();
  switch ( c)
   {
    case '1': // 1 Start assisted breathing
#ifdef serialVerbose
     Serial.println(F("Breathing started"));
#endif
active=true;
    break;
    case 'C': // CV / CP mode
     if (ch=='V') CVmode=true; 
     if (ch=='P') CVmode=false; 
#ifdef serialVerbose
     Serial.println((CVmode)?F("CV mode"):F("CP mode"));
#endif
    break;
    case '0': // 0 Stop assisted breathing
     active=false;
#ifdef serialVerbose
     Serial.println(F("WARNING: Breathing stopped"));
#endif
     setDisplayMenu(Stop_message,std_dly,0);
    break;
    case 'S': // S set Speed (in BPM)
     if (ok=((r>=minBPM) && (r<=maxBPM)))
      {reqBPM=r; prstat();} 
#ifdef serialVerbose
        else Serial.println(F("Out of range (6-40BPM)"));  
#endif
    break;
    case 'V': // V set Volume (in liters)
     r=r*1000;
     if (ok=((r>=minVolume) && (r<=maxVolume)))
      {reqVolume=r; prstat();}
#ifdef serialVerbose
      else Serial.println(F("Out of range (0.1-1.5l)"));  
#endif
    break;
    case 'P': // P Set pressure (in KPa)
     r=r*1000;
     if (ok=((r>=minCompression) && (r<=maxCompression)))
      {reqCompression=r; prstat();} 
#ifdef serialVerbose
      else Serial.println(F("Out of range (1-20KPa)"));  
#endif
    break;
    case 'E': // E set expiration ratio (x inspiration)
     if (ok=((r>=minExpirationRatio) && (r<=maxExpirationRatio)))
      {expirationRatio=r; prstat();}
#ifdef serialVerbose
      else Serial.println(F("Out of range (1 to 3 x inspiration)"));  
#endif
    break;
    case 'Y': // Y set sYnc (in % of cycle)
     r=r/100;
     if (ok=((r>=minSyncRatio) && (r<=maxSyncRatio)))
      {syncRatio=r; prstat();}
#ifdef serialVerbose
      else Serial.println(F("Out of range (0-40%)"));  
#endif
    break;
    case 'M': // M show Measurements from the sensor
     meas();  
    break;
#ifdef serialTelemetry    
    case 'T': // T toggle Telemetry data on/off\n
     telemetry=!telemetry;  
    break;
#endif
#ifdef E2PROM
    case 'W': // T Terminal mode (human readable)
     eeput(0);  // Save settings to EEPROM
     menuItem=0;
#ifdef serialVerbose
     Serial.println(F("Parameters Written"));  
#endif
    break;
#endif    
    case 'D': // D debug (on / off)
     debug=!debug;
    break;
    //case 'R': // R Reset
    // cli();
    // resetFunction();
    //break;
    case '?': // help
     if (term)
      {
#ifndef ArduinoPlotter        
#ifdef serialVerbose
       Serial.println(F("0 Stop assisted breathing\n"
                        "1 Start assisted breathing\n"
                        "CV sets Constant Volume mode\n"
                        "CP sets Constant Pressure mode\n"
                        "S set Speed (in BPM)\n"
                        "V set Volume (in liters)\n"
                        "P set Pressure (in KPa)\n"
                        "E set expiration ratio (x inspiration)\n"
                        "Y set sYnc (in % of cycle)\n"
                        "M show Measurements from the sensor\n"
                        "T toggle Telemetry data on/off\n"
#ifdef E2PROM
                        "W Write parameters to permanent memory\n"
#endif       
                        "D debug (on / off)\n"
                        //"R Reset\n"
                        "? print this\n" )); 
#else
       Serial.println(F("CMD: 0 1 CV CP S V P E Y M T"
#ifdef E2PROM
                        " W"
#endif       
                        " D"
                        //"R Reset\n"
                        " ?")); 
#endif
#endif
      }
      break;
    default:   // Unknown command 
#ifdef serialVerbose 
     Serial.println(F("Unknown command. type '?' for instructions.")); 
#else
     Serial.println(F("?")); 
#endif
    break;
   } 
 if (!ok) Serial.println('?'); else prstat();
#ifdef watchdog
 Watchdog.reset();
#endif        
}

void SerialWait(int t)
{
 while (t--) 
  {
   delayMicroseconds(10); // 10 usec delay
   while (Serial.available())
    {       
      char inChar = (char)Serial.read();
      if (inChar != '\r') { 
      //Serial.print(inChar); 
      if (inChar == '\n') 
      {
        Buffer += (char)0; 
       SerialCommand();
       BufferLength=CommandMaxLength; // force buffer reset
      } 
      else
      {
       Buffer += inChar;
       BufferLength++;
      }
      if (BufferLength>=CommandMaxLength) {BufferLength=0;Buffer = "";}  // Overflow buffer reset
      }
    }
  }
}
#endif

/*
 *   This is the state machine that manages all the data displayed, the menus and animations.
 *   
 *   There are lots if conditional code that make it less legible but allow to change the display 
 *   controller while preserving the navigation logic
 */

#ifdef dispMenus
void displayMenu()  
{
#ifdef PCF8574LCDDisplay  
 doDisp=true;  // will update the display by default
 byte dispt=dispTick % 3;
 switch (dispPhase) {
  case 0:
     strcpy(disp,"OPEN  SOURCE\n VENTILATOR");
     dispPhase++;
    break;
  case 1:
     strcpy(disp,"Software V0.20");
     dispDelay=100;
     dispPhase=100;
    break;
  case 10:
     sprintf(disp, "ERROR %2d\n", dispParam);
     strcat(disp,(dispParam==0)?"NO SENSOR":"UNKNOWN SENSOR");     
     dispPhase=10;
  case 100:
     sprintf(disp,"S%02d V%d P%d \n",int(BPM),int(volume),int(compression/100));
     dispDelay=100;  
     if (active) dispPhase=(dispParam==0)?100:103; else dispPhase++;
    break;
  case 101:
     strcpy(disp,"\nSTART ");
     strcat(disp,(CVmode)?"VC":"PC");
     dispPhase=(dispParam==0)?100:103;
     dispDelay=80;
    break;
  case 102:
     strcpy(disp,"\nSTOP");
     dispPhase=100;
     dispDelay=50;
    break;
  case 103:
     sprintf(disp,"\nTEMP %dc   ",int(temperature)); 
     dispPhase++;
     dispDelay=80;
    break;
  case 104:
     sprintf(disp,"\nHumi %d    ",int(humidity));    
     dispPhase++;
    break;
  case 105:
     sprintf(disp,"\nPRESS. %dRH ",int(relPressure/100));
     dispPhase++;
    break;
  case 106:
     sprintf(disp,"\nSYNC.  %d  ",int(syncRatio*100.1));
     dispPhase++;
    break;
  case 107:
     sprintf(disp,"\nRatio %d.%d  ",int(expirationRatio),int (expirationRatio*10) %10);
     dispPhase=100;
    break;
      
  case 110:
     sprintf(disp,"\nSET Rate  %d  ",int(reqBPM));
     dispDelay=60;  
    break;
  case 120:
     sprintf(disp,"\nSET Volume %d",int(reqVolume)); // character N looks better chan character M
     dispDelay=60;  
    break;
  case 130:
     sprintf(disp,"\nSET Press. %d",int(reqCompression/100));
     dispDelay=60;  
    break;
  case 140:
     sprintf(disp,"\nSET Sync. %2d    ",int(syncRatio*100.1)); // 100.1 instead of 100 prevents rounding errors
     dispDelay=60;  
    break;
  case 150:
     dispTick++;
     sprintf(disp,"\nSET Ex ratio %d.%d",int(expirationRatio),int (expirationRatio*10) %10); 
     dispDelay=60;  
    break;
  case 160:
     strcpy(disp,"\nSave Setup     ");
     dispDelay=60;  
    break;
  default:
    if (term) Serial.println(dispPhase); 
    dispPhase=0; // Abnormal the phase should always be defined
   break;
  }
 dispCnt=dispDelay;
#endif 

#ifdef TM1638Display  
 doDisp=true;  // will update the display by default
 byte dispt=dispTick % 3;
 switch (dispPhase) {
  case 0:
     strcpy(disp,"OPEN");
     dispPhase++;
    break;
  case 1:
     strcpy(disp,"SOURCE");
     dispPhase++;
    break;
  case 2:
     strcpy(disp,"VENTILAT");
     dispDelay=30;
     dispPhase++;
    break;
  case 3:
     strcpy(disp,"ENTILATO");
     dispDelay=20;
     dispPhase++;
    break;
  case 4:
     strcpy(disp,"NTILATOR");
     dispDelay=60;
     dispPhase++;
    break;
  case 5:
     strcpy(disp,"SOFT 0.20");
     dispDelay=100;
     dispPhase=100;
    break;
  case 10:
     sprintf(disp, "ERROR %2d", dispParam);
     dispPhase+=dispParam+1; // Jump to correct error 
    break;
  case 11:
     strcpy(disp,"NO.SENSOR");
     dispPhase=10;
    break;
  case 12:
     strcpy(disp,"UNKNOWN");
     dispPhase=99;
    break;
  case 99:
     strcpy(disp,"SENSOR");
     dispPhase=10;
    break;
  case 100:
     if (compression<10000) 
       sprintf(disp,"%02d %d.%d %d.%d",int(BPM),int(volume/1000),int (volume/100) %10,int(compression/1000),int (compression/100) %10);
      else
       sprintf(disp,"%02d %d.%d %d",int(BPM),int(volume/1000),int (volume/100) %10,int(compression/1000));
     dispDelay=100;  
     if (active) dispPhase=(dispParam==0)?100:103; else dispPhase++;
    break;
  case 101:
     strcpy(disp,"START  _");
     disp[6]=(CVmode)?'V':'P';
     dispPhase=(dispParam==0)?100:103;
     dispDelay=80;
    break;
  case 102:
     strcpy(disp,"STOP   _");
     dispPhase=100;
     dispDelay=50;
    break;
  case 103:
     sprintf(disp,"TENP %dc",int(temperature)); // character N looks better chan character M
     dispPhase++;
     dispDelay=80;
    break;
  case 104:
     sprintf(disp,"Huni%d",int(humidity));    // character N looks better chan character M
     dispPhase++;
    break;
  case 105:
     sprintf(disp,"PRESS. %dH",int(relPressure/100));
     dispPhase++;
    break;
  case 106:
     sprintf(disp,"SYNC.  %d",int(syncRatio*100.1));
     dispPhase++;
    break;
  case 107:
     sprintf(disp,"Ratio %d.%d",int(expirationRatio),int (expirationRatio*10) %10);
     dispPhase=100;
    break;
      
  case 110:
     sprintf(disp,(dispTick++ & 1)?"SET   %d":"RATE  %d",int(reqBPM));
     dispDelay=60;  
    break;
  case 120:
     sprintf(disp,(dispTick++ & 1)?"SET  %d":"VOL.  %d",int(reqVolume)); // character N looks better chan character M
     dispDelay=60;  
    break;
  case 130:
     sprintf(disp,(dispTick++ & 1)?"SET   %d.%d":"PRESS. %d",int(reqCompression/100));
     dispDelay=60;  
    break;
  case 140:
     sprintf(disp,(dispTick++ & 1)?"SET   %2d":"SYNC.  %2d",int(syncRatio*100.1)); // 100.1 instead of 100 prevents rounding errors
     dispDelay=60;  
    break;
  case 150:
     dispTick++;
     sprintf(disp,"\nSET EXP Ratio  %d.%d",int(expirationRatio),int (expirationRatio*10) %10); 
     dispDelay=60;  
    break;
  case 160:
     strcpy(disp,(dispTick++ & 1)?"SETUP  _":"SAVE   _");
     dispDelay=60;  
    break;
  default:
    if (term) Serial.println(dispPhase); 
    dispPhase=0; // Abnormal the phase should always be defined
   break;
  }
 dispCnt=dispDelay;
#endif 
}
#endif


void Timer() 
{
 timerLvl++;  
 if (!stopMove) stepper.run();
 if (timerLvl>1)  // only does the motor update if nested more than 1 level
  {
   timerLvl--; 
  }
  else
  {
#ifdef Led  
   digitalWrite(pin_LED, HIGH);
#endif
   sei();                        // Critical : Allows nested interrupts
   switch (timCnt % 50) {        // 10 ms tick period for interface handling
#ifdef dispMenus     
    case 0:
     if (--dispCnt<=0) displayMenu();
     break;
    case 10:                     // change the display
     if (doDisp)
      {
       if  (I2CBusy)
         timCnt-=5;  // retry next time
        else
         {
          doDisplay(&display); 
          doDisp=false;
         } 
      }
     break;
#endif     
#ifdef Beeper       
    case 15:                     // update the beep
     if (beepCnt==1)
      {
        digitalWrite(pin_Beep, HIGH);  // inactive (inverted)
        alarm=0;
      }
     if (beepCnt) beepCnt--;
     break;
#endif     
#ifdef dispBargraph 
    case 20:                     // update the bargraph
     displayBargraph((breathGraphCnt>0)?barGraph-bargraphDot:barGraph);
     if (breathGraphCnt) breathGraphCnt--;
     break;
#endif     
#ifdef isKeyboard
    case 30:                     // update the keyboard
      debounceKeyboard();
     break;
#endif     
    } 
   if (++timCnt>9999) timCnt=0;  // counts 0 to 9999
#ifdef Led
   digitalWrite(pin_LED, LOW);
#endif
#ifdef watchdog
   Watchdog.reset();
#endif        
   cli();  // no interrupts
   timerLvl--; 
  }
}


void __attribute__ ((noinline)) IIRFilter (float raw,float &flt,float pct)
{
 flt=(flt==0)?raw:flt*(1-pct)+raw*pct;
}


void readSensors() // Works no matter if sensor is present or not (returns default values if not present)

{
 float temp(NAN), hum(NAN), pres(NAN);

// Add your sensor reading routines here 

#ifdef BoschBMxSensor 
 BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
 BME280::PresUnit presUnit(BME280::PresUnit_Pa);
 I2CBusy=true;
 bme.read(pres, temp, hum, tempUnit, presUnit);
#endif

#ifdef BoschBMP180Sensor 
float amb(NAN);
if ((sc % 8)==1)   // every 400 ms, launch a new measurement
  {
   BMP180Phase=false; 
   if (!bmp180.measureTemperature()) 
    {
     bmp180.begin(); // Restarts the sensor whenever an error is detected
#ifdef debug_3
     Serial.print('!');
#endif       
    }
  }
 if (bmp180.hasValue())
  {
    if (!BMP180Phase)
     {
      temperature2=bmp180.getTemperature();
      BMP180Phase=bmp180.measurePressure();
      if (!BMP180Phase) 
       {
        bmp180.begin(); // Restarts the sensor whenever an error is detected
#ifdef debug_3
        Serial.print('!');
#endif       
       }
      
     }
#ifdef debug_3
    Serial.print('+');
#endif  
    if (BMP180Phase)
     {     
      amb=bmp180.getPressure();
      if (!isnan(amb) && (amb>minAtmosphericPressure) &&  (amb<maxAtmosphericPressure))
       {
#ifdef debug_3
         Serial.print(amb);
#endif       
        IIRFilter(amb,ambientPressure,ambientPressureFilter);
        // ambientPressure=(ambientPressure==0)?amb:ambientPressure*(1-ambientPressureFilter)+amb*ambientPressureFilter; // low pass filtering
        lastAmbientPressure=millis();
       }
     }  
  }
#endif
 I2CBusy=false;

 if (isnan(temp) || isnan(pres))  // If data was not correctly acquired
 {
  temp=temperature;  // assume it did not move
  if (presM1>presM2)  // pressure was increasing 
    pres=presM1+2*(presM1-presM2); // assume the pressure increase is 2x what it was
   else
    pres=presM1;  // Pressure was decreasing, assume it stays constant
   
#ifdef debug_3
  Serial.print("E");
#endif       
 }
 else
 {
  presM2=presM1;
  presM1=pres;
 }
#ifdef debug_3
   tab();
   Serial.print((sensPressure)?"1 ":"0 ");
   tab();
   Serial.print((sensHumidity)?"1 ":"0 ");
   tab();
   Serial.print(pres);
   tab();
   Serial.print(temp);
   tab();
   Serial.print(hum);
   tab();
   Serial.println(ambientPressure);
#endif
#ifdef tempSensor
  if (sensTemperature)
   {
    temperature=temp;
   }
  else 
#endif
    temperature=defaultTemperature;
#ifdef humiditySensor
  if (sensHumidity)
   {
    humidity=hum;
   }
  else
#endif
    humidity=defaultHumidity;
#ifdef pressureSensor
  if (sensPressure)
   {
     pressure=pres;
#ifdef TwoPressureSensors
     // approximate atmospheric pressure by averaging when the bag is filled and the machine does not run.
     if ((stepper.currentPosition()==0) && !active && ((millis()>lastAmbientPressure+delayAmbientPressure) || !sensAmbientPressure))  
      IIRFilter(pressure,ambientPressure,ambientPressureFilter);
      //ambientPressure=(ambientPressure==0)?pressure:ambientPressure*(1-ambientPressureFilter)+pres*ambientPressureFilter; // low pass filtering
#else
     if ((stepper.currentPosition()==0) && !active)  // approximate athmospheric pressure by averaging when the bag is filled and the machine does not run.
      IIRFilter(pressure,ambientPressure,ambientPressureFilter);
      //ambientPressure=(ambientPressure==0)?pressure:ambientPressure*(1-ambientPressureFilter)+pres*ambientPressureFilter; // low pass filtering
#endif     
      IIRFilter(pressure,avgPressure,avgPressureFilter);
      //avgPressure=(avgPressure==0)?pressure:avgPressure*(1-avgPressureFilter)+pres*avgPressureFilter;                      // low pass filtering
     peakPressure=pressure-avgPressure;
   }
   else
#endif
   {
    pressure=defaultPressure;
    ambientPressure=defaultAmbientPressure;
   }
  if (ambientPressure==0) ambientPressure=defaultAmbientPressure; 
  relPressure=pressure-ambientPressure; 
 
#ifdef debug_4
   Serial.print(pressure);
   tab();
   Serial.print(temperature);
   tab();
   Serial.print(humidity);
   tab();
   Serial.println(ambientPressure);
#endif  
#ifdef CurrentSense
 if (sensCurrent)
  {
   //set_sleep_mode(SLEEP_MODE_IDLE);
   //sleep_enable();
   //sleep_mode(); 
   //sleep_disable();
   int c=analogRead(pin_current_Sense);
   if (stepper.speed()==0) // average of idle current value when the motor does not run
   // idleCurrent=(idleCurrent==0)?c:idleCurrent*(1-idleCurrentFilter)+c*idleCurrentFilter; // low pass filtering
   IIRFilter(c,idleCurrent,idleCurrentFilter);
   currentVal=c-int(idleCurrent); 
   //currentRaw=(currentRaw==0)?analogCurrentRatio*currentVal:currentRaw*(1-currentFilter)+analogCurrentRatio*currentVal*currentFilter; // low pass filtering
   IIRFilter(analogCurrentRatio*currentVal,currentRaw,currentFilter);
   current=abs(currentRaw);
  }
  else current=0;
#endif
#ifdef debug_2  // print data for 1 in 10 measurement cycles
 if ((term) && ((sc % 10)==2) && (debug))
   meas();
#endif
sc++; 
}


void detectSensors()  // Detects the various sensors - initialisation for new sensors should be added here
{
 sensHumidity=false;
 sensTemperature=false;
 sensPressure=false;
#ifdef CurrentSense
 sensCurrent=false;
#endif

// Add your sensor detection routines here
 I2CBusy=true;
#ifdef BoschBMxSensor
 if(bme.begin())
  {
   switch(bme.chipModel())
    {
    case BME280::ChipModel_BME280:
      if (term) Serial.println(F("BME280"));
      sensHumidity=true;
      sensTemperature=true;
      sensPressure=true;
      break;
    case BME280::ChipModel_BMP280:
      if (term) Serial.println(F("BMP280"));
      sensTemperature=true;
      sensPressure=true;
      break;
    default:
      setDisplayMenu(error_message,std_dly,1);
      if (term) Serial.println(F("UNKNOWN sensor"));
    }
  }
  else
  {
   setDisplayMenu(error_message,std_dly,0);
   if (term) Serial.println(F("No BME280 sensor"));
  }
#endif  

#ifdef BoschBMP180Sensor
if (sensAmbientPressure=bmp180.begin())
  {
    if (term) Serial.println(F("BMP180"));
    bmp180.resetToDefaults(); //reset sensor to default parameters.
    bmp180.setSamplingMode(BMP180MI::MODE_UHR); //enable ultra high resolution mode for pressure measurements
  }
  else
  if (term) Serial.println(F("No BMP180 sensor"));
#endif
 I2CBusy=false;
#ifdef CurrentSense
 pinMode(pin_current_Sense,INPUT_PULLUP);
 delay (10);
 sensCurrent=(analogRead(pin_current_Sense)<750); // If the pin is left floating, it's voltage should be well above midpoint
 if (sensCurrent)
  {
   currentVal=abs(analogRead(pin_current_Sense)-current0);
   sensCurrent=(currentVal < currentMaxIdle);
  }
 if (sensCurrent)  Serial.println(F("Found current sensor!"));
#endif
}

#ifdef PCF8574LCDDisplay
  LiquidCrystal_PCF8574 display;
#endif

void setup() {
 // put your setup code here, to run once:
 timerLvl=0;
 Serial.begin(SERIAL_BAUD);
 while (!Serial && millis()<3000); // wait for USB Serial ready. Needed for native USB (non blocking)
 term=Serial; 
 if (term) Serial.println(F("Open Source Ventilator V 0.20"));
#ifdef jm_Wire
 twi_readFrom_wait = true; // twi_readFrom() waiting loop
 twi_writeTo_wait = true;  // twi_writeTo() waiting loop
#endif 
#ifdef I2C
 Wire.begin();
#endif
  
  display = initDisplay(&Wire);
  
 Timer1.initialize(200);  
 Timer1.attachInterrupt(Timer);
#ifdef watchdog
#ifndef  watchdogProtect        
 Watchdog.enable(watchdogDelay);  // if the watchdog is enabled without delayed start, let it go
                                  // (See the warning about bricking your board before enabling this)
 Watchdog.reset();
#endif        
#endif        
 
 setDisplayMenu(startup_message); 
 
 pinMode(pin_Stepper_DIR, OUTPUT);
 digitalWrite(pin_Stepper_DIR, LOW);     // active (inverted)
 pinMode(pin_Stepper_Step, OUTPUT);
 digitalWrite(pin_Stepper_Step, LOW);    // active (inverted)
#ifdef disableMotorctrl
 pinMode(pin_Stepper_Disable, OUTPUT);
 digitalWrite(pin_Stepper_Disable, LOW);  // active (inverted)
 motorState=true;
#endif 
 detectSensors();
 eeget ();    // read startup parameters (either from EEPROM or default value)
#ifdef USBcontrol 
 Buffer="?";  // send informations to the serial port
 SerialCommand();
 Buffer="";
#endif 
 BPM=reqBPM;                 // Start from these values without sweep
 volume=reqVolume;
 compression=reqCompression;
 readSensors();   
 tick=millis();
 breathLength=int(60000/BPM);
 breathe==tick-breathLength;
 stopMove=false;
// stepper.setMaxSpeed(3500);
// stepper.setAcceleration(20000);
// stepper.moveTo(0);
#ifdef Beep
 beep(20,0); 
#endif
#ifdef ArduinoPlotter
 telemetry=true;
 active=true;
#endif
}



void limitValues(float &B,float &V,float &C,float &D,float &E) // prevent values from going outside of their respective limits 
{
 B = constrain(B, minBPM, maxBPM); 
 V = constrain(V, minVolume, maxVolume); 
 C = constrain(C, minCompression, maxCompression); 
 D = constrain(D, minSyncRatio, maxSyncRatio); 
 E = constrain(E, minExpirationRatio, maxExpirationRatio); 
}

#ifdef isKeyboard

void pressOk()
{
 if (menuItem==0)
   {
    active=!active;  // Toggle function when "Enter" key pressed
    if (!active) setDisplayMenu(Stop_message,std_dly,0);
   }
  else
   {                 // Return to main screen when "Enter" key pressed
#ifdef E2PROM            
    if (menuItem==MenuItems-1) eeput(0);  // Save settings to EEPROM
#endif           
    menuItem=0;
    setDisplayMenu(Menu_message,std_dly,0);
   }
 if ((active) && (stepper.distanceToGo() == 0))  // If the motor is idle, start immediately
  {
   breathe=millis()-breathLength; // will happen immediately
  }
}

void pressPrevNext()
{
 if ((keys[btnPrev]) && (menuItem>0)) {menuItem--;setDisplayMenu(Menu_message+10*menuItem,std_dly,0);}            // Previous menu item (no rollover)
 if ((keys[btnNext]) && (menuItem<MenuItems-1)) {menuItem++;setDisplayMenu(Menu_message+10*menuItem,std_dly,0);}  // Next menu item (no rollover) 
}

void pressUpDown()
{
 int n=0;
 if (keys[btnDn]) n=-1;
 if (keys[btnUp]) n=1;
 switch(menuItem)
  {
   case 0:
      if (n<0) dispParam=(dispParam==0)?1:0;  // button Dn triggers extended data
      if (n>0) CVmode=!CVmode;                // button Up triggers control mode change
      if (dispParam) setDisplayMenu(Measurement_Message);
      if (n==1) setDisplayMenu(Measurement_Start);
      dispCnt=0;
     break;
   case 1:
      reqBPM=reqBPM+(n*stepBPM);
     break;
   case 2:
      reqVolume=reqVolume+(n*stepVolume);
     break;
   case 3:
      reqCompression=reqCompression+(n*stepCompression);
     break;
   case 4:
      syncRatio=syncRatio+(n*stepSyncRatio);
     break;
   case 5:
      expirationRatio=expirationRatio+(n*stepExpirationRatio);
     break;
  }
 limitValues(reqBPM,reqVolume,reqCompression,syncRatio,expirationRatio); 
}

#endif

#ifdef sweep
/*
 *  When set values are changed, actual values are progressively changed from the current to the new value
 */
void sweepValues()  
{
 float f=max(reqBPM,BPM)*maxBPMchange;  // slowly change BPM value to reach requested BPM value
 limitValues(BPM,volume,compression,syncRatio,expirationRatio);
 if (BPM>reqBPM)
   {
    BPM=BPM-f;
    if (BPM<reqBPM) BPM=reqBPM;
   } else
   {
    BPM=BPM+f;
    if (BPM>reqBPM) BPM=reqBPM;
   }
 f=max(reqVolume,volume)*maxVolumeChange;  // slowly change volume value to reach requested volume value        
 if (volume>reqVolume)
   {
    volume=volume-f;
    if (volume<reqVolume) volume=reqVolume;
   } else
   {
    volume=volume+f;
    if (volume>reqVolume) volume=reqVolume;
   }  
 f=max(reqCompression,compression)*maxCompressionChange;  // slowly change ompression value to reach requested pressure value
 if (compression>reqCompression)
   {
    compression=compression-f;
    if (compression<reqCompression) compression=reqCompression;
   } else
   {
    compression=compression+f;
    if (compression>reqCompression) compression=reqCompression;
   }  
}
#endif 


/*
 *  When the set pressure is reached or if the pressure gets too high (depending on the mode of operation)
 *  the patient inspiration phase is ended and the motor stopped fast.
 *  After overshoot, the motor comes back to the exact position where it was when the routine was called
 */

void motorFastStop()
{
 stepper.setAcceleration(motorMaxAcceleration);
 stepper.moveTo(stepper.currentPosition ());  // overshoot, then return
}

#ifdef serialTelemetry
void basicTelemetry(boolean b)
{
 if (active)
  {
   if (!Mactive)
    {
     startActive=millis();
     Mactive=true;
    }
   if (telemetry)
    {
     if (breathPhase==0)
#ifdef ArduinoPlotter     
      Serial.println("Phase,Alarm,MotorPos,motorTarget,Pressure,AmbPress,Volume,Target,BreathLng,BreathIn,BreathInSpeed,BreathInAccel,BreathOut,BreathOutSpeed,BreathOutAccel"); 

     //Serial.print(float(millis()-startActive)/1000);
     //sep();
     Serial.print(breathPhase*50);
     sep();
     Serial.print(alarm*20);
     sep();
     Serial.print(arduinoPlotterMotorOffset+stepper.currentPosition());
     sep();
     Serial.print(arduinoPlotterMotorOffset+stepper.targetPosition ());
     sep();
     Serial.print(arduinoPlotterPressureOffset+relPressure*0.010197162129779282*10);
#else
      Serial.println("Time,Phase,Alarm,Motor pos,Pressure,Ambient press,Volume,Target,Breath lng,Breath in,Breath in speed,Breath in accel,Breath out,Breath out speed,Breath out accel"); 
     Serial.print(float(millis()-startActive)/1000);
     sep();
     Serial.print(breathPhase);
     sep();
     Serial.print(alarm);
     sep();
     Serial.print(stepper.currentPosition());
     sep();
     Serial.print(relPressure*0.010197162129779282);
#endif
     if (b) Serial.println();
    }
   }
  else
   Mactive=false;
}
#endif



/*
 *  This function is called once at the start of every breath.
 *  
 *  It's main role is to establish the timing for the whole cycle as well as the motor acceleration 
 *  and projected top speed.
 */

void breathingCycleStart()
{
#ifdef sweep
 if (!allSet) sweepValues();      // Manage the ramp up / ramp down of parameters
#else
 BPM=reqBPM;
 volume=reqVolume;
 compression=reqCompression;
#endif 
 breathPhase=0;
 breathLength=int(60000/BPM); 
 breathe=millis();    // time for the next cycle start
 syncDelay=breathLength*syncRatio;
 breathTime=breathLength-syncDelay;
 breathIn=breathTime/(1+expirationRatio);
 breathOut=breathTime-breathIn;
 breathInSpeed=volume/breathIn;
 breathOutSpeed=volume/breathOut;
 breathInAcceleration=breathInSpeed*breathInSpeed*motorAcceleration;
 breathOutAcceleration=breathOutSpeed*breathOutSpeed*motorAcceleration;
 motorTarget=volume*motorVolumeRatio;
#ifdef serialTelemetry
 if (active)
  {
   if (telemetry)
    {
     basicTelemetry(false); 
     sep();
#ifdef ArduinoPlotter     
     Serial.print(arduinoPlotterOffset+long(ambientPressure)/1000);
     sep();
     Serial.print(arduinoPlotterOffset+int(volume));
     sep();
     Serial.print(arduinoPlotterOffset+long(volume*motorVolumeRatio));
     sep();
     Serial.print(arduinoPlotterOffset+float(breathLength)/10);
     sep();
     Serial.print(arduinoPlotterOffset+float(breathIn)/10);
     sep();
     Serial.print(arduinoPlotterOffset+int(breathInSpeed*motorSpeed)/10);
     sep();
     Serial.print(arduinoPlotterOffset+float(breathInAcceleration)/50);
     sep();
     Serial.print(arduinoPlotterOffset+float(breathOut)/10);
     sep();
     Serial.print(arduinoPlotterOffset+int(breathOutSpeed*motorSpeed)/10);
     sep();
     Serial.println(arduinoPlotterOffset+float(breathOutAcceleration)/50);
#else     
     Serial.print(long(ambientPressure));
     sep();
     Serial.print(int(volume));
     sep();
     Serial.print(long(volume*motorVolumeRatio));
     sep();
     Serial.print(float(breathLength)/1000);
     sep();
     Serial.print(float(breathIn)/1000);
     sep();
     Serial.print(int(breathInSpeed*motorSpeed));
     sep();
     Serial.print(float(breathInAcceleration)/1000);
     sep();
     Serial.print(float(breathOut)/1000);
     sep();
     Serial.print(int(breathOutSpeed*motorSpeed));
     sep();
     Serial.println(float(breathOutAcceleration)/1000);
#endif     
     }
   }
#endif
#ifdef debug_0   // print data at the start of each cycle
 Serial.print(breathLength);
 tab();
 Serial.print(breathTime);
 tab();
 Serial.print(breathIn);
 tab();
 Serial.print(breathOut);
 tab();
 Serial.print(volume);
 tab();
 Serial.print(breathInSpeed);
 tab();
 Serial.println(breathOutSpeed);
#endif
 breathGraphCnt=10;              // blinks the LED at the start of a cycle   
 stepper.setMaxSpeed(int(breathInSpeed*motorSpeed));
 stepper.setAcceleration(breathInAcceleration);
 if (active)  // When a real cycle is done (with the motor actually moving)
  {
#ifdef disableMotorctrl // Optional ability to disable the motor after a time of inactivity
   if (!motorState)     // Useful to allow changing the ambubag and preserving energy on battery
    {
     digitalWrite(pin_Stepper_Disable, LOW);  // activate
     motorState=HIGH;
     delay(motorDriverRecovery);  // Allow some time for the motor driver to recover
    }
#endif     
#ifdef Beeper
  beep(2,0);
#endif        
  breathPhase=1;
  stepper.moveTo(motorTarget);        
  }
}


/*
 * This function is called periodically during the patient inspiration phase
 * 
 * This is the place where the pressure buildup is to be measured and,
 * if required, the motor can be slowed down or stopped.
 * the inspiration phase can also be aborted prematurely if the pressure
 * gets too high.
 * 
 * This routine also manages the various errors that can happen
 * 
 * The interval between calls is determined by samplePeriod (default 50 ms)
 * 
 */

void breathingInspiration ()
{
 if (!CVmode)  // CP (Pressure Control) Mode
  {
   if ((relPressure>compressionScale) && (breathPhase==1))  // Check for pressure
    {
     motorFastStop();
     if (stepper.currentPosition ()<(motorTarget)*failVolumeRatio) beep(300,1); // Insufficient volume alarm
    }
  }
}

/*
 *  This function is called periodically at all times 
 *  
 *  It detects if the pressure is high
 */


void breathOverpressureAlarm()
{
 if (relPressure>alarmCompressionValue*compressionScale) // over pressure detected as an error in all modes if it lasts more than 500 ms 
  {
   if (highPressureAlarmCnt<highPressureAlarmDetect)
    highPressureAlarmCnt++; 
#ifdef Beeper        
   else 
    beep(300,2); 
#endif
   if (stepper.currentPosition()< stepper.targetPosition ())
     motorFastStop(); // stop the motor if it was moving forward (increasing the pressure)
  }  
 else
  if (highPressureAlarmCnt) highPressureAlarmCnt--;
}

/*
 * This function is called once when the patient inspiration phase is over
 * and the motor needs to return to it's zero point.
 * 
 * It sets the motor acceleration and projected top speed for the movement.
 */

void breathingExpirationStart()
{
 if (breathPhase<2)
  {breathPhase++; 
#ifdef serialTelemetry
   basicTelemetry(true);
#endif        
   stepper.setMaxSpeed(int(breathOutSpeed*motorSpeed));
   stepper.setAcceleration(breathOutAcceleration);
   stepper.moveTo(0);
  } 
}


void breathAssistTrigger()
{
 if ((relPressure<-0.1*compressionScale) && (peakPressure<-0.1*compressionScale))
  {
   if (breathPhase==3)
    {
     breathPhase=4;
#ifdef serialTelemetry
     if (active) basicTelemetry(true);
#endif        
    }
   breathe=millis()-breathLength; // Trigger a new cycle sooner.
#ifdef debug_1    // print data for user triggered cycle    
   Serial.print(relPressure);
   tab();
   Serial.print(peakPressure);
   tab();
   Serial.print(compressionScale);
   Serial.println(F(" Sync"));
#endif        
  }
}


void loop() {

//    stepper.run();

  if (millis()>tick+samplePeriod)  // Read the sensors and acct according to the results (every 50 millisecond)
   {
    tick=tick+samplePeriod;
    readSensors();
#ifdef serialTelemetry
   if (active) basicTelemetry(true);
#endif
#ifdef analogKeyboard
   analogKeys=analogRead(pin_analogKeyboard);            // Buffered analog value for keyboard
#ifdef debugAnalogKeyboard
   Serial.print(F("Analog Key Value: "));
   Serial.print(analogKeys); 
   Serial.print(F("\t Tolerance (5%): "));
   Serial.println(int(float(analogKeys) / 20)+1); 
   analogKeys=0;  // Does not transmit keys when debugging the analog values.
#endif
#endif

#ifdef sweep
    allSet=((reqBPM==BPM) && (reqVolume==volume) && (reqCompression==compression));
#endif
    compressionScale=(compression>minCompression)?compression:defaultCompression;
    volumeScale=(volume>minVolume)?volume:defaultVolume;   
    if (breathPhase==1) breathingInspiration ();
    breathOverpressureAlarm();
#ifdef dispBargraph   
    updateBargraph(CVmode, stepper.currentPosition());   
#endif      
#ifdef isKeyboard      
    // Keyboard actions
    if (keys[btnOk]   && !mkeys[btnOk])   pressOk();        // Ok button
    if (keys[btnPrev] && !mkeys[btnPrev]) pressPrevNext();  // previous item button
    if (keys[btnNext] && !mkeys[btnNext]) pressPrevNext();  // next item button
    if (keys[btnDn]   && !mkeys[btnDn])   pressUpDown();    // lower value button
    if (keys[btnUp]   && !mkeys[btnUp])   pressUpDown();    // increase value button
    byte ct=maxBtn;
    while (ct--) 
     {
      if (keys[ct]) lastKey=millis();
      mkeys[ct]=keys[ct];
     }
#ifdef dispMenus     
    if ((millis()>lastKey+keyIdleReturn) && (menuItem!=0))
     {
      menuItem=0;
      setDisplayMenu(Menu_message,std_dly,0);
     }
#endif     
#endif     
#ifdef disableMotorctrl                  // after being inactive for a while, the motor is disabled
     if (active) lastActive=millis();
     if (millis()>lastActive+motorOffDelay) 
      {
       digitalWrite(pin_Stepper_Disable, HIGH);  // Deactivate
       motorState=LOW; 
      }
#endif     
   }
  if ((millis()>breathe+breathLength) && (stepper.currentPosition()>0)) breathe=millis()-breathLength+50; // wait for the previous cycle to finish (safety)
   else
   {
    if (millis()>breathe+breathLength) breathingCycleStart(); // start of a cycle
    if ((millis()>breathe+breathIn) && (stepper.distanceToGo() == 0)) breathingExpirationStart(); // second half of a cycle
    if ((syncDelay) && ((millis()>breathe+breathLength-syncDelay+20) || ((breathPhase==2) && (stepper.distanceToGo() == 0))))  // Synchronization period
     {
      breathPhase=3;
      breathAssistTrigger();
     }
   }  
#ifdef watchdog
#ifdef watchdogProtect        
 if (!watchdogStarted && (millis()>watchdogStartDelay))       // True when the watchdog has been activated. the first few seconds after reset are without watchdog protection
  {
   Watchdog.enable(watchdogDelay);
   watchdogStarted=true; 
  }
#endif        
 Watchdog.reset();
#endif        
#ifdef USBcontrol     
 SerialWait(1);          // wait for 10 usec looking for received characters
#else
 delayMicroseconds(10);  // wait for 10 usec
#endif 
}
