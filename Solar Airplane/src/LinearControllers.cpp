//
// LinearControllers.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode+ 
// http://embedXcode.weebly.com
//
// Project 		Solar Airplane
//
// Created by 	Sage Thayer, 1/24/16 9:08 PM
// 				Sage Thayer
//
// Copyright 	(c) Sage Thayer, 2016
// Licence		Beerware: Use this software as you wish.
//              If you like it, buy me a beer somtime :)
//
// See 			LinearControllers.h and ReadMe.txt for references
//


// Library header
#include "LinearControllers.h"

// Code

// Constructors and overloads
PIDController::PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki) {
    //constructor overload. set private vars
    dt = differentialTime;
    itsKp = Kp; itsKd = Kd; itsKi = Ki;   // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
}

PIDController::PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki, int desiredOutput) {
    //constructor overload. set private vars
    dt = differentialTime;
    itsKp = Kp; itsKd = Kd; itsKi = Ki;   // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
    itsDesiredOutput = desiredOutput;
}

PIDController::PIDController(unsigned long differentialTime, float Kp, float Kd, float Ki, int desiredOutput, int feedback) {
    //constructor overload. set private vars
    dt = differentialTime;
    itsKp = Kp; itsKd = Kd; itsKi = Ki;   // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
    itsDesiredOutput = desiredOutput;
    itsFeedback = feedback;
}

// Methods

void PIDController::setGains(float Kp, float Kd, float Ki) {
    itsKp = Kp; itsKd = Kd; itsKi = Ki; // controller gains:: Kp: porportional, Kd: derivative, Ki: integral
}

void PIDController::setDesiredOuptut(int desiredOutput) {
    itsDesiredOutput = desiredOutput;
}
void PIDController::setFeedback(int feedback) {
    itsFeedback = feedback;
}

void PIDController::getControlSignal(int *controlSignal) {
    
    // calculate errors
    itsPorportionalError = itsDesiredOutput - itsFeedback;
    itsDerivativeError = itsPorportionalError/dt;
    
    itsIntegralError = itsIntegralError + (itsPorportionalError * dt); // compute the integral
    
    //put all together
    *controlSignal = -itsKp*itsPorportionalError - itsKd*itsDerivativeError - itsKi*itsIntegralError;
    
}
