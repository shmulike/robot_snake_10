/**
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "PID.h"

using namespace std;

class PIDImpl
{
public:
    PIDImpl( double dt, double max, double min, double Kp, double Ki, double Kd );
    ~PIDImpl();
    double calculate( double setpoint, double pv );
    void resetSum();
    void setK(double dt, double max, double min, double Kp, double Ki, double Kd );
    void testp(int);


private:
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;
};

//PID::PID(){}
PID::PID( double dt=0, double max=0, double min=0, double Kp=0, double Ki=0, double Kd=0 )
{
    std::cout << "Run constractor" << std::endl;
    pimpl = new PIDImpl(dt,max,min,Kp,Ki,Kd);
    std::cout << "done constractor" << std::endl;

}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
void PID::resetSum(){
    pimpl->resetSum();
}
void PID::setK(double dt, double max, double min, double Kp, double Ki, double Kd ){
    std::cout << "1"<< std::endl;
    pimpl->setK( dt,  max,  min,  Kp,  Ki,  Kd);
    std::cout << "3"<< std::endl;
}
int PID::test(int x){
    pimpl->testp(x);
    return x*2;
}
PID::~PID()
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Ki, double Kd ) :
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

double PIDImpl::calculate( double setpoint, double pv )
{
    std::cout << "calc --> _Kp: " << _Kp << std::endl;
    // Calculate error
    double error = setpoint - pv;
    // Proportional term
    double Pout = _Kp * error;
    // Integral term
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
    _pre_error = error;

    return output;
}

void PIDImpl::resetSum(){
    _integral = 0;
}

void PIDImpl::setK( double dt, double max, double min, double Kp, double Ki, double Kd  ){
    _dt = dt;
    _max = max;
    _min = min;

    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;

    std::cout << "new _Kp: " << _Kp << std::endl;
}

void PIDImpl::testp(int x) {
    _Kp = 5.6;
    std::cout << "new _Kp: " << _Kp << std::endl;
}
PIDImpl::~PIDImpl()
{
}

#endif