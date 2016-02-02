///
/// @file		Flight.h
/// @brief		Library header
/// @details	<#details#>
/// @n	
/// @n @b		Project Solar Airplane
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Sage Thayer
/// @author		Sage Thayer
///
/// @date		1/24/16 9:12 PM
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

#include "LinearControllers.h"
#include "Vector.h"

#ifndef Flight_cpp
#define Flight_cpp

// Rigid body Abstract Data Type
// Rigid body Abstract Data Type
class RigidBody {
public:
    //constructors
    RigidBody();
    virtual ~RigidBody(){};
    
    //virtual methods to overload
    virtual void setPosition(Vector3<float>& a) = 0;
    virtual void setVelocity(Vector3<float>& a) = 0;
    virtual void setAcceleration(Vector3<float>& a) = 0;
    
    virtual void setAttitude(Vector3<float>& a) = 0;
    virtual void setAngularRates(Vector3<float>& a) = 0;
    virtual void setAngularAccelerations(Vector3<float>& a) = 0;
    
    virtual void setInertias(int& mass, int& momentOfInertia) = 0;
    
    //accessors
protected:
    //inertial Dynamics
    int Mass;   //in grams
    int I;      //in grams*cm^2
    
    // linear dynamics
    Vector3<float> position;    //x,y,z
    Vector3<float> velocity;      //u,v,w
    Vector3<float> acceleration;
    
    //rotation dynamics
    Vector3<float> attitude;
    Vector3<float> angular_rates;
    Vector3<float> angular_acc;
    
};

class UAV : public RigidBody{
public:
    //constructors
    UAV();
    ~UAV();
    
    //methods
    void yaw(int yaw_rate);
    void pitch(int roll_rate);
    void roll(int pitch_rate);
    
    void steadyLevelFlight();
    
    void circle();
    void circle(int altitude);
    
private:
    int itsSpeed, itsAltitude;
};



#endif
