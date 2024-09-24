//
// Created by fogoz on 15/09/2024.
//

#ifndef TEENSYCODE_POSITION_H
#define TEENSYCODE_POSITION_H


class Position {
    double x;
    double y;
    double angle;
public:
    Position(double x, double y, double angle);

    double getX();

    double getY();

    double getAngle();

    void setX(double x);

    void setY(double y);

    void setAngle(double angle);
};


#endif //TEENSYCODE_POSITION_H
