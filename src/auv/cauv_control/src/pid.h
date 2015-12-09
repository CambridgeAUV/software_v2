/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#pragma once

#include <ctime>

#include <utility/time.h>

namespace cauv {

class PIDControl
{
    public:
        double Kp,Ki,Kd,scale;
        double Ap, Ai, Ad, thr;
        double KpMAX, KpMIN, KdMAX, KdMIN, KiMAX, KiMIN;
        double Kp1, Ki1, Kd1;
        double errorMAX;
        bool is_angle;
        bool enabled;

        PIDControl(bool is_angle);
        ~PIDControl(){}
        void initialise(double _Kp, double _Ki, double _Kd, double _scale,
                        double _Ap, double _Ai, double _Ad, double _thr,
                        double _errorMAX);
        double get_demand(double target, double current);
        void reset();

    private:
        double integral, previous_derror, previous_mv;

        double error, previous_error;

        TimeStamp time_current_signal;
        TimeStamp time_previous_signal;

        double getErrorAngle(double const& target, double const& current);
        double getError(double const& target, double const& current);
        double smoothedDerivative();
};

} // namespace cauv
