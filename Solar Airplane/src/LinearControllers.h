///
/// @file		LinearControllers.h
/// @brief		Library header
/// @details	<#details#>
/// @n	
/// @n @b		Project Solar Airplane
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Sage Thayer
/// @author		Sage Thayer
///
/// @date		1/24/16 9:08 PM
/// @version	<#version#>
/// 
/// @copyright	(c) Sage Thayer, 2016
/// @copyright	Beerware: Use this software as you wish.
//              If you like it, buy me a beer somtime :)
///
/// @see		ReadMe.txt for references
///


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#   include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#   include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#   include "libpandora_types.h"
#   include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#   include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#   include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#   include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#   include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#   include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#   include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#   include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#   include "Arduino.h"
#elif defined(SPARK) // Spark specific
#   include "application.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#   include "Arduino.h"
#else // error
#   error Platform not defined
#endif // end IDE

#ifndef LinearControllers_cpp
#define LinearControllers_cpp

//------------------------------------------------------------------------------//
//-----------------------General PID controller object--------------------------//
//------------------------------------------------------------------------------//

template <class TT> class PIDController {
public:
    //constructors and destructor
    PIDController(){itsKp = 0; itsKd = 0; itsKi = 0;}
    PIDController(unsigned long& differentialTime){itsKp = 0; itsKd = 0; itsKi = 0; dt = differentialTime;}
    PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki);
    PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki, TT desiredOutput);
    PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki, TT desiredOutput, TT feedback);
    ~PIDController(){};
    
    //methods
    void setGains(float Kp, float Kd, float Ki);
    void setDesiredOuptut(TT desiredOutput);
    void setFeedback(TT input);
    void setDifferentialTime(unsigned long& Dt) {dt = Dt;}
    
    void update();
    TT getControlSignal() { return itsControlSignal;}
    
private:
    TT itsControlSignal;
    float itsKp, itsKd, itsKi;   // controller gains:: Kp: position, Kd: derivative, Ki: integral
    TT itsDesiredOutput, itsFeedback;
    TT itsPorportionalError;
    TT itsDerivativeError, itsIntegralError;
    unsigned long dt;
    
};


#endif
