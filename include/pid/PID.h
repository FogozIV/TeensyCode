//
// Created by fogoz on 26/06/2024.
//

#ifndef TEENSYCODE_PID_H
#define TEENSYCODE_PID_H
#include "Arduino.h"

namespace control{
    class BasicPID{
    public:
        virtual double calculateValue(double error) = 0;
    };

    class PID : BasicPID{
    protected:
        double previousError = 0;
        double integral = 0;
        double kp;
        double ki;
        double kd;
        double antiWindup;
        double period;
    public:
        PID(double kp, double ki, double kd, double antiWindup, double period);
        double calculateValue(double error) override;
    };

    class FilteredPID : PID{
    protected:
        double lowPassFilterPeriod;
        double previousD = 0;
    public:
        FilteredPID(double kp, double ki, double kd,
                    double antiWindup, double period, double lowPassFilterPeriod);
        double calculateValue(double error) override;
    };
}

#endif //TEENSYCODE_PID_H
