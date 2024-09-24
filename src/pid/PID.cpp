//
// Created by fogoz on 26/06/2024.
//
#include "pid/PID.h"


namespace control
{
    PID::PID(double kp, double ki, double kd, double antiWindup, double period) : kp(kp), ki(ki), kd(kd), antiWindup(antiWindup), period(period){

    }

    double PID::calculateValue(double error) {
        double result = 0;
        result += kp * error;
        integral += ki * error * period;
        integral = max(min(antiWindup, integral), -antiWindup);
        result += integral;
        result += kd * (previousError - error) / period;
        previousError = error;
        return result;
    }

    FilteredPID::FilteredPID(double kp, double ki, double kd,
                             double antiWindup, double period, double lowPassFilter) : PID(kp, ki, kd, antiWindup, period), lowPassFilterPeriod(lowPassFilter) {

    }

    double FilteredPID::calculateValue(double error) {
        double result = 0;
        result += kp * error;
        integral += ki * error * period;
        integral = max(min(antiWindup, integral), -antiWindup);
        result += integral;
        double periodRatio = lowPassFilterPeriod/period;
        double D = periodRatio/(1+periodRatio) * previousD + kd * 1/(1+periodRatio) * (error - previousError)/period;
        result += D;
        previousD = D;
        previousError = error;
        return result;
    }
}
