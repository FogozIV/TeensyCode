//
// Created by fogoz on 15/09/2024.
//

#include "utils/Position.h"

Position::Position(double x, double y, double angle) : x(x), y(y), angle(angle){

}

double Position::getX() {
    return x;
}

double Position::getY() {
    return y;
}

double Position::getAngle() {
    return angle;
}

void Position::setX(double x) {
    Position::x = x;
}

void Position::setY(double y) {
    Position::y = y;
}

void Position::setAngle(double angle) {
    Position::angle = angle;
}
