/**
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "PID.h"

using namespace std;

//PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Ki, double Kd ) :
PID::PID( double dt, double max, double min, double Kp, double Ki, double Kd ) :
        _dt(dt),
        _max(max),
        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0),
        _integral(0)
{
}

PID::~PID()
{
}

//double PIDImpl::calculate( double setpoint, double pv )
double PID::calculate( double setpoint, double pv )
{
    // Calculate error
    double error = setpoint - pv;
    // Proportional term
    double Pout = _Kp * error;
    // Integral term
    if (_pre_error / error < 0)
        _integral = 0;          // Clamping
    else
        _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    this->_pre_error = error;

    // Debug
    //std::cout << "calc: Iout" << Iout << endl;
//    std::cout << "error: " << error << "\tPout: " << Pout << "\tDout: " << Dout << "\tIout: " << Iout << "\toutput: " << output << endl;


    return output;
}

void PID::resetSum(){
    this->_integral = 0;
}
double PID :: getsum(){
    return _integral;
}
void PID::setK( double dt, double max, double min, double Kp, double Ki, double Kd  ){
    this->_dt = dt;
    this->_max = max;
    this->_min = min;

    this->_Kp = Kp;
    this->_Kd = Kd;
    this->_Ki = Ki;

    // Debug
    std::cout << "setK: Kp" << this->_Kp << endl;
}

#endif