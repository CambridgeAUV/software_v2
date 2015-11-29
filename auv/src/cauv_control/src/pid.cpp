/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "pid.h"
#include <utility/math.h>
#include <utility/rounding.h>
#include <boost/make_shared.hpp>

#include <ros/node_handle.h>

#define CAUV_DEBUG_COMPAT
#include <debug/cauv_debug.h>

using namespace cauv;

bool cauv::check_lock_token(TokenLock &lock, const cauv_control::ControlToken &token) {
    boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::universal_time();
    if ((current_time - lock.current_tok_time).total_milliseconds() < lock.current_token.timeout) {
        //token has not timed out
        if (token.priority < lock.current_token.priority) {
            return false; //lower priority always loses
        }
        if (token.priority == lock.current_token.priority &&
            token.token > lock.current_token.token) {
            return false; //use token (which should be unique, and is currently the start time of the process) as tiebreaker
        }
    }
    //New lock token has won (either from same process or higher priority process, or timeout)
    if (lock.current_token.token != token.token) {
        info() << "Lock Token" << token.token << "(Priority"
               << token.priority << ")won over token" << lock.current_token.token;
    }
    lock.current_token = token;
    lock.current_tok_time = current_time;
    return true;
}

//TODO: merge this into time.h (There's a conflicting definition in
//mapping/stuff.h which returns seconds
static float operator-(TimeStamp const& l, TimeStamp const& r)
{
    int secs_delta = l.secs - r.secs;
    float msecs_delta = (l.musecs - r.musecs) / 1000.0f;
    return 1000 * secs_delta + msecs_delta;
}

PIDControl::PIDControl(std::string topic, boost::shared_ptr<TokenLock> lock_, bool is_angle_)
        : target(0),
          Kp(1), Ki(1), Kd(1), scale(1),
          Ap(1), Ai(1), Ad(1), thr(1),
          KpMAX(1), KpMIN(0), KdMAX(1), KdMIN(0), KiMAX(1), KiMIN(0),
          Kp1(1), Ki1(1), Kd1(1),
          errorMAX(1000),
          is_angle(is_angle_), enabled(false),
          integral(0), previous_derror(0), previous_mv(0),
          retain_samples_msecs(1000),
          token_lock(lock_)
{
    ros::NodeHandle h;
    params_sub = h.subscribe(topic + "params", 10, &PIDControl::onParamsMessage, this);
    target_sub = h.subscribe(topic + "target", 1,  &PIDControl::onTargetMessage, this);
    state_pub = h.advertise<cauv_control::PIDState>(topic + "state", 5);
    if (!state_pub) {
        throw std::runtime_error("Empty State Publisher!");
    }
}

void PIDControl::onTargetMessage(const cauv_control::PIDTarget::Ptr &m) {
    if (check_lock_token(*token_lock, m->token)) {
        if (m->enabled && !enabled) {
            reset();
        }
        enabled = m->enabled;
        target = m->target;
    }
}

void PIDControl::onParamsMessage(const cauv_control::PIDParams::Ptr &m) {
    Kp       = m->Kp;
    Ki       = m->Ki;
    Kd       = m->Kd;
    scale    = m->scale;
    Ap       = m->Ap;
    Ai       = m->Ai;
    Ad       = m->Ad;
    thr      = m->thr;
    errorMAX = m->maxError; //I'm special!
}

void PIDControl::reset()
{
    integral = 0;
    previous_errors.clear();
    previous_derror = 0;
    previous_mv = 0;
    previous_time.secs = 0;
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

double PIDControl::smoothedDerivative()
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
}

double PIDControl::getMV(double current)
{
    double error;
    if(is_angle)
        error = getErrorAngle(target, current);
    else
        error = getError(target, current);
    error = clamp(-errorMAX, error, errorMAX);

    if (previous_time.secs == 0) {
        previous_time = now();
        previous_errors.push_back(std::make_pair(previous_time, error));
        return 0;
    }

    TimeStamp tnow = now();
    previous_errors.push_back(std::make_pair(tnow, error));
    while((tnow - previous_errors.front().first) > retain_samples_msecs)
        previous_errors.pop_front(); 

    double dt = tnow - previous_time; // dt is milliseconds
    previous_time = tnow;

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

    cauv_control::PIDState msg;
    msg.mv = previous_mv;
    msg.error = error;
    msg.derror = Kd1 * de;
    msg.ierror = Ki1 * integral;
    msg.Kp = Kp1;
    msg.Kd = Kd1;
    msg.Ki = Ki1;

    state_pub.publish(msg);

    return previous_mv;
}
