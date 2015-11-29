/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#pragma once

#include <deque>
#include <vector>
#include <string>

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <cauv_control/PIDState.h>
#include <cauv_control/PIDParams.h>
#include <cauv_control/PIDTarget.h>
#include <cauv_control/ControlToken.h>

#include <utility/time.h>
#include <utility/inbox.h>

#include <ros/publisher.h>

namespace cauv {

struct TokenLock {
    cauv_control::ControlToken current_token;
    boost::posix_time::ptime current_tok_time;
};

bool check_lock_token(TokenLock &lock, const cauv_control::ControlToken &token);

class PIDControl
{
    public:
        double target;
        double Kp,Ki,Kd,scale;
        double Ap, Ai, Ad, thr;
        double KpMAX, KpMIN, KdMAX, KdMIN, KiMAX, KiMIN;
        double Kp1, Ki1, Kd1;
        double errorMAX;
        bool is_angle;
        bool enabled;

        PIDControl(std::string topic, boost::shared_ptr<TokenLock> token_lock_, bool is_angle);
        double getMV(double current);
        void reset();

    private:
        double integral, previous_derror, previous_mv;
        std::deque< std::pair<TimeStamp, double> > previous_errors;
        TimeStamp previous_time;
        int retain_samples_msecs;

        void onTargetMessage(const cauv_control::PIDTarget::Ptr &m);
        void onParamsMessage(const cauv_control::PIDParams::Ptr &m);

        double getErrorAngle(double const& target, double const& current);
        double getError(double const& target, double const& current);
        double smoothedDerivative();

        boost::shared_ptr<TokenLock> token_lock;

        ros::Subscriber params_sub;
        ros::Subscriber target_sub;
        ros::Publisher state_pub;
};

} // namespace cauv
