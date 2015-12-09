/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "pid.h"

#include <ctime>

#include <utility/time.h>
#include <utility/rounding.h>
#include <utility/math.h>

// #define CAUV_DEBUG_COMPAT
// #include <debug/cauv_debug.h>

using namespace cauv;

PIDControl::PIDControl(bool is_angle_)
        : Kp(1), Ki(1), Kd(1), scale(1),
          Ap(1), Ai(1), Ad(1), thr(1),
          KpMAX(1), KpMIN(0), KdMAX(1), KdMIN(0), KiMAX(1), KiMIN(0),
          Kp1(1), Ki1(1), Kd1(1),
          errorMAX(1000),
          is_angle(is_angle_), enabled(false),
          dt(0),
          error(0), previous_error(0),
          ierror(0),
          derror(0),
          demand(0)
{}

void PIDControl::initialise(double _Kp, double _Ki, double _Kd, double _scale,
                double _Ap, double _Ai, double _Ad, double _thr,
                double _errorMAX)
{
    Kp       = _Kp;
    Ki       = _Ki;
    Kd       = _Kd;
    scale    = _scale;
    Ap       = _Ap;
    Ai       = _Ai;
    Ad       = _Ad;
    thr      = _thr;
    errorMAX = _errorMAX;
}

void PIDControl::reset()
{
    dt = 0;
    error = 0;
    previous_error = 0;
    ierror = 0;
    derror = 0;
    demand = 0;
    time_previous_signal.secs = 0;
    time_previous_signal.musecs = 0;
}

double PIDControl::getErrorAngle(double const& target, double const& current)
{
    double diff = mod(target - current, 360.0);
    if(diff >  180) diff -= 360;
    if(diff <= -180) diff += 360;
    return diff;
}

double PIDControl::getError(double const& target, double const& current)
{
    return target - current;
}

double PIDControl::get_demand(double target, double current)
{
    // Update the previous error "history" variable
    previous_error = error;

    // If first iteration, then just update time and bail out
    if (time_previous_signal.secs == 0) {
        time_previous_signal = now();
        return 0;
    }

    // Calculate dt and update time
    dt = 1000*(now() - time_previous_signal); // dt is in milliseconds
    time_previous_signal = now();

    // Calculate error
    if(is_angle)
        error = getErrorAngle(target, current);
    else
        error = getError(target, current);
    error = clamp(-errorMAX, error, errorMAX);

    // Calculate error integral
    ierror += error*dt;
    ierror = clamp(-errorMAX, ierror, errorMAX);

    // Calculate error derivative
    derror = derivative_smoothing_coef * (error - previous_error) / dt
                - (1 - derivative_smoothing_coef) * derror;  // We want times in milliseconds
    derror = clamp(-errorMAX, derror, errorMAX);
   
    
    // variable gains
    KpMAX=Kp*Ap;
    KpMIN=Kp/Ap;
    KiMAX=Ki*Ai;
    KiMIN=Ki/Ai;
    KdMAX=Kd*Ad;
    KdMIN=Kd/Ad;
    
    Kp1 = (KpMAX - KpMIN)*abs(error)/thr + KpMIN;
    Ki1 = (KiMAX - KiMIN)*abs(error)/thr + KiMIN;
    Kd1 = KdMAX;
    
    if (abs(error)>0.00001) {
        Kd1=( KdMIN - KdMAX )*abs(error)/thr + KdMAX;
    }
    
    if (abs(error)>thr) {
        Kp1=KpMAX;
        Ki1=KiMAX;
        Kd1=KdMIN;
    }

    //Control action
    demand =  scale * (Kp1 * error + Ki1 * ierror + Kd1 * derror);

    // cauv_control::PIDState msg;
    // msg.mv = demand;
    // msg.error = error;
    // msg.derror = Kd1 * derror;
    // msg.ierror = Ki1 * ierror;
    // msg.Kp = Kp1;
    // msg.Kd = Kd1;
    // msg.Ki = Ki1;

    // state_pub.publish(msg);

    return demand;
}
