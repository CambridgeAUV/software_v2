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
          integral(0), previous_derror(0), previous_mv(0),
          error(0), previous_error(0)
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
    integral = 0;
    error = 0;
    previous_error = 0;
    previous_derror = 0;
    previous_mv = 0;
    time_current_signal.secs = 0;
    time_current_signal.musecs = 0;
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

/*double PIDControl::smoothedDerivative()
{
    int n_derivatives = 0;
    double derivative_sum = 0;

    if(!previous_errors.size()){
        warning() << "no derivative samples available";        
        return 0.0;
    }

    for(int i = 0;i < int(previous_errors.size())-1; i++) {
        float dt_msecs = (previous_errors[i+1].first - previous_errors[i].first);
        if(dt_msecs != 0) {
            derivative_sum += (previous_errors[i+1].second - previous_errors[i].second) / dt_msecs;
            n_derivatives++;
        } else {
            error() << "controller update frequency too high";
        }
        if(dt_msecs < 2) {
            warning() << "controller update frequency < 2ms";
        }
    }
    if(!n_derivatives) {
        warning() << "no derivative samples used";
        return 0.0;
    }
    return derivative_sum / n_derivatives;

    // For an alternative way, see https://books.google.co.uk/books?id=BcnwAAAAQBAJ&pg=PA103&lpg=PA103&dq=control+theory+derivative+smoothing&source=bl&ots=DbjmoYVpSj&sig=5DXqqcATMb0oIndoaNLeuLzMQ-4&hl=en&sa=X&ved=0ahUKEwi8n_qpubbJAhXGWBQKHej6DM8Q6AEIMTAC#v=onepage&q=control%20theory%20derivative%20smoothing&f=false
}*/

double PIDControl::smoothedDerivative()
{
    return (error - previous_error)/(1000* (time_current_signal - time_previous_signal)); // We want times in milliseconds
}

double PIDControl::get_demand(double target, double current)
{
    if (time_current_signal.secs == 0) {
        time_current_signal = now();
        return 0;
    }

    time_previous_signal = time_current_signal;
    previous_error = error;
    time_current_signal = now();

    if(is_angle)
        error = getErrorAngle(target, current);
    else
        error = getError(target, current);
    error = clamp(-errorMAX, error, errorMAX);

    double dt = 1000*(time_current_signal - time_previous_signal); // dt is in milliseconds
    integral += error*dt;
    integral = clamp(-errorMAX, integral, errorMAX);

    double de = smoothedDerivative();
    de = clamp(-errorMAX, de, errorMAX);
    previous_derror = de;
    
    
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
    previous_mv =  scale * (Kp1 * error + Ki1 * integral + Kd1 * de);

    // cauv_control::PIDState msg;
    // msg.mv = previous_mv;
    // msg.error = error;
    // msg.derror = Kd1 * de;
    // msg.ierror = Ki1 * integral;
    // msg.Kp = Kp1;
    // msg.Kd = Kd1;
    // msg.Ki = Ki1;

    // state_pub.publish(msg);

    return previous_mv;
}
