/**

 */

#ifndef _PID_H_
#define _PID_H_

class PID
{
public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    //PID();
    PID( double dt, double max, double min, double Kp, double Ki, double Kd );

    // Returns the manipulated variable given a setpoint and current process value
    double calculate( double setpoint, double pv );
    void resetSum();
    double getsum ();
    void setK(double dt, double max, double min, double Kp, double Ki, double Kd);
    ~PID();

private:
    //PIDImpl *pimpl;
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;
};

#endif
