#ifndef OSV_CONFIG_H
#define OSV_CONFIG_H

//************************   DEVICE OPERATIONAL PARAMETERS   ************************  

/*
       WARNING : When changing min and max value, manually change the text in the SerialCommand procedure accordingly

       The values are taken from various documents including :
       https://www.gov.uk/government/publications/coronavirus-covid-19-ventilator-supply-specification/rapidly-manufactured-ventilator-system-specification
       https://docs.google.com/document/d/1h77FkKXqPOwVqfOj-PIJSjYX9QiEx69Av2gYuzqZolw/edit#

       some changes have been made to havee a lower default volume in case the machine is used 
       with an infant to prevent damaging their lungs with an adult setting.
*/

#define minBPM                     10.0   // minimum respiratory speed
#define defaultBPM                 20.0   // default respiratory speed
#define stepBPM                     1.0   // adjustment step for respiratory speed
#define maxBPM                     35.0   // maximum respiratory speed
#define maxBPMchange                0.2   // maximum respiratory speed change in proportion of final value per beat (1=100%)
#define minVolume                 100.0   // minimum respiratory volume in milliliters 
#define defaultVolume             400.0   // default respiratory volume in milliliters 
#define stepVolume                 20.0   // adjustment step for respiratory volume in milliliters 
#define maxVolume                 800.0   // maximum respiratory volume in milliliters 
#define maxVolumeChange             0.25  // maximum respiratory volume change in proportion of final value per beat (1=100%) 
#define minCompression           1000.00  // minimum compression for the ambu-bag in Pa
#define stepCompression           500.00  // adjustment step for compression for the ambu-bag in Pa
#define defaultCompression       3000.00  // default compression for the ambu-bag in Pa
#define maxCompression          20000.00  // maximum compression for the ambu-bag in Pa
#define maxCompressionChange        0.5   // maximum compression for the ambu-bag change in proportion of final value per beat (1=100%)
#define minSyncRatio                0.00  // minimum portion of the cycle for breath synchronisation
#define stepSyncRatio               0.05  // adjustment step for portion of the cycle for breath synchronisation
#define defaultSyncRatio            0.15  // default portion of the cycle for breath synchronisation
#define maxSyncRatio                0.40  // maximum portion of the cycle for breath synchronisation
#define minExpirationRatio          1.00  // minimum portion of the cycle for breath synchronisation
#define stepExpirationRatio         0.2   // adjustment step for portion of the cycle for breath synchronisation
#define defaultExpirationRatio      1.2   // default portion of the cycle for breath synchronisation
#define maxExpirationRatio          3.00  // maximum portion of the cycle for breath synchronisation
#define failVolumeRatio             0.90  // In CP mode, Alarm if pressure reached before that portion of programmed volume.



#define ambientPressureFilter       0.02  // IIR filtering ratio (lower value produce longer time constant)
#define avgPressureFilter           0.1   // IIR filtering ratio (lower value produce longer time constant)
#define defaultPressure        101325.00  // Pressure in Pa returned when no sensor is found (1 atm)
#define defaultAmbientPressure 101325.00  // assumed ambient pressure in Pa returned when no sensor is found (1 atm)
#define defaultHumidity            50.00  // humidity in % RH returned when no sensor is found
#define defaultTemperature         20.00  // temperature in °C returned when no sensor is found
#define alarmCompressionValue       1.1   // if the pressure exceeds the presed value * alarmCompressionValue, 
                                          // then trigger an alarm and stop the motor if needed  
#define minAtmosphericPressure 60000     // minimum atmospheric pressure that would be considered valid                                           
#define maxAtmosphericPressure 120000    // maximum atmospheric pressure that would be considered valid                                           



/*******************************   MOTOR PARAMETERS   *******************************
 *    
 *     These values will be highly dependant on mechanical design.
 *     When modifying the values, always check with an oscilloscope that the movements completes    
 *     without overlaps and without too much idle time. 
 *     Also check that the motor can properly follow the speed acceleration required under load.
 *     Wrong values can cause unpredictables moves and motor stalls.
 *     The worst case scenario is at max BPM, with max volume, max sync period and max ratio
 *     With default parameters, the whole compression can become as short as 250 ms
 */

#define motorSpeed               5000     // Speed for 1 liter/second
#define motorAcceleration        8500     // Acceleration for 1 liter / second (inverse square of flow)
#define motorMaxAcceleration    30000     // deceleration when pressure limit reached
#define motorVolumeRatio          0.8     // Ratio of distance in steps to air volume in step per milliliter.
                                          

/*******************************   HARDWARE OPTIONS   *******************************
 *    
 *    It's normal for the program to not compile if some of these are undefined as they need an alternative
 *    
 */

#define USBcontrol         // USB / serial Command line interface.
                           // This allows complete control of all parameters.

#define E2PROM             // Uses the internal EEPROM for parameter storage

//#define TM1638Keyboard     // Use a TM1638 for the keyboard

#define analogKeyboard

//#define TM1638Display      // Use a TM1638 for the display

#define PCF8574LCDDisplay  // Use a 2x16 characters LCD with a PCF8574 'backpack' as a display

#define TM1638bargraph     // Use a TM1638 for the Led pressure bargraph

#define ActiveBeeper       // Active beeper can be used on any pin. Passive beeper will require a PWM capable pin

#define Led                // Led debugging / Signal (should be disabled if SPI peripherals are present)

#define BoschBMxSensor     // Bosch Sensortech BMP280 or BME280

//#define BoschBMP180Sensor  // Bosch Sensortech BMP180 

//#define TwoPressureSensors // Double pressure sensor (one for barometric pressure)

#define stepDirMotor       // Control the motor with step and Direction signals

#define disableMotorctrl   // control moter activation / desactivation      

//#define CurrentSense     // uncomment to add motor current sensing with the Allegromicro ACS712 sensor
                           // This is not enabled by default as first test show the sensor is not sensitive
                           // enough. Another sensor should probably be used.

// # define watchdog       // WARNING: Do not enable this setting  while debugging, you might brick 
                           // your dev. board. If this happens, you'll need to flash not just the 
                           // bootloader, but the whole program space.
                           // The problem is worse in Nanos with a bootloader that does not clear
                           // the watchdog settings. 

#define serialTelemetry    // Sends real time data about time, current breathing phase, motor position, pressure 
                           // while the machine works in CSV format


//************************************   DEBUGGING   *************************************

// #define debug_0      // print data for each cycle
// #define debug_1      // print data for user triggered cycle    
// #define debug_2      // print data for 1 in 10 measurement cycles
// #define debug_3      // print sensor datadata 
// #define debug_4      // temperature / humidity / pressure 

//#define debugAnalogKeyboard

#define ArduinoPlotter  // This will turn on telemetry automatically
                        // With this feature on, absolute pressure and motor speeds are scaled.  


//******************************   IMPLIED DEFINITIONS  ********************************

#ifdef BoschBMxSensor
#ifndef I2C
#define I2C
#endif
#endif

#ifdef BoschBMP180Sensor
#ifndef I2C
#define I2C
#endif

#define dispMenus     // Alternate displays must also enable this
#endif

#ifdef PCF8574LCDDisplay
#ifndef I2C
#define I2C
#endif
#define dispMenus     // Alternate displays must also enable this
#endif

#ifdef  TM1638Keyboard
#define TM1638
#define isKeyboard    // Alternate keyboards must also enable this
#endif

#ifdef analogKeyboard
#define isKeyboard    // Alternate keyboards must also enable this
#endif

#ifdef  TM1638Display
#ifndef TM1638
#define TM1638
#endif
#define dispMenus     // Alternate displays must also enable this
#endif

#ifdef  TM1638bargraph
#ifndef TM1638
#define TM1638
#endif
#define dispBargraph  // Alternate physical (LED) bargraphs must also enable this
#endif


#ifdef  ActiveBeeper
#define Beeper
#endif

#ifdef  watchdog
#define watchdogProtect          // This prevents the watchdog from being fired before watchdogDelay  
                                 // To allow an new software to be uploaded     
#define watchdogDelay      1000  // Maximum time before the watchdog times out and resets 
#define watchdogStartDelay 8000  // Delay before the watchdog becomes active (used to have the time
                                 // to reload the program if it hangs the controller. Only for development.
#endif


#ifndef analogKeyboard
#undef debugAnalogKeyboard
#endif

#ifdef debugAnalogKeyboard
#undef serialTelemetry
#undef ArduinoPlotter
#endif

#ifdef ArduinoPlotter            // The plotter needs telemetry data
#ifndef serialTelemetry
#define serialTelemetry
#endif
#define arduinoPlotterPressureOffset 600  // Y axis offset for the pressure curve
#define arduinoPlotterOffset         800  // Y offset for the data displayed once per cycle
#define arduinoPlotterMotorOffset    25   // Y offset for the motor position (prevents shifts if undershoot)
#undef debug_0
#undef debug_1
#undef debug_2
#undef debug_3
#undef debug_4
#undef debug_5
#undef debug_6
#undef debug_7
#undef debug_8
#undef debug_9
#undef debug_10
#undef debug_11
#undef debug_12
#undef debug_13
#undef debug_14
#undef debug_15
#undef debug_16
#undef debug_17
#undef debug_18
#undef debug_19
#endif

#endif /* !OSV_CONFIG_H */
