//
// Created by fogoz on 15/09/2024.
//

#ifndef TEENSYCODE_ROBOT_H
#define TEENSYCODE_ROBOT_H


#include <QuadEncoder.h>
#include <FS.h>
#include "utils/Position.h"
#include "utils/pinModeEX.h"
#include "ArduinoJson.h"

#ifndef SELECTED_CHIP
#define SELECTED_CHIP BUILTIN_SDCARD
#endif

class Robot {
private:
    double left_wheel_diam = 1;
    double right_wheel_diam = 1;
    double track_mm = 100;
    int32_t previous_encoder_left;
    int32_t previous_encoder_right;

    double evaluatedLeft = 0.0;
    double evaluatedRight = 0.0;
    Position position = Position(0, 0, 0);

    QuadEncoder* encoderLeft = nullptr;
    QuadEncoder* encoderRight = nullptr;
    bool sd_present = false;
    JsonDocument jsonData;
public:
    Robot(uint8_t pinA, uint8_t pinB, uint8_t pinA2, uint8_t pinB2, uint8_t channel1=1, uint8_t channel2=2);

    void update();

    void setPos(double x, double y, double a);

    void setEncoder(int32_t left, int32_t right);

    void setTrackMm(double trackMm);

    void setRightWheelDiam(double rightWheelDiam);

    void setLeftWheelDiam(double leftWheelDiam);

    std::tuple<uint8_t*, size_t> getRawData();
private:
    void update_position();




};


#endif //TEENSYCODE_ROBOT_H
