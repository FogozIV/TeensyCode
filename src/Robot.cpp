//
// Created by fogoz on 15/09/2024.
//

#include <SD.h>
#include "Robot.h"

Robot::Robot(uint8_t pinA, uint8_t pinB, uint8_t pinA2, uint8_t pinB2, uint8_t channel1, uint8_t channel2) {
    if (!SD.begin(SELECTED_CHIP)) {
        sd_present = false;
    } else {
        sd_present = true;
    }
    File dataFile = SD.open("encoder.json", FILE_READ);
    if (dataFile) {
        deserializeJson(jsonData, dataFile);
        if (jsonData["left_wheel_diam"].is<double>()) {
            left_wheel_diam = jsonData["left_wheel_diam"].as<double>();
        }
        if (jsonData["right_wheel_diam"].is<double>()) {
            right_wheel_diam = jsonData["right_wheel_diam"].as<double>();
        }
        if (jsonData["track_mm"].is<double>()) {
            track_mm = jsonData["track_mm"].as<double>();
        }
    }
    QuadEncoder *encoder1 = new QuadEncoder(channel1, pinA, pinB);
    QuadEncoder *encoder2 = new QuadEncoder(channel2, pinA2, pinB2);
    pinMode({pinA, pinA2, pinB, pinB2}, INPUT);
    encoder1->setInitConfig();
    encoder2->setInitConfig();
    encoder1->init();
    encoder2->init();

    encoderLeft = encoder1;
    encoderRight = encoder2;

    previous_encoder_left = encoderLeft->read();
    previous_encoder_right = encoderRight->read();

}

void Robot::update() {
    update_position();
}

void Robot::update_position() {
    int32_t encoderLeftValue = encoderLeft->read();
    int32_t encoderRightValue = encoderRight->read();
    double left = encoderLeftValue - previous_encoder_left;
    double right = encoderRightValue - previous_encoder_right;
    previous_encoder_right = encoderRightValue;
    previous_encoder_left = encoderLeftValue;

    double dx, dy, da;
    left *= this->left_wheel_diam;
    right *= this->right_wheel_diam;
    evaluatedLeft = left;
    evaluatedRight = right;
    double angle = (right - left) / 2;
    double distance = (right + left) / 2;
    if (angle == 0) {
        dx = cos(position.getAngle()) * distance;
        dy = sin(position.getAngle()) * distance;
        da = 0;
    } else {
        double r = (double) distance * track_mm / ((double) angle * 2);
        double arc_angle = 2 * angle / track_mm;

        dx = r * (-sin(position.getAngle()) + sin(position.getAngle() + arc_angle));
        dy = r * (cos(position.getAngle()) - cos(position.getAngle() + arc_angle));
        da = arc_angle;
    }
    position.setX(position.getX() + dx);
    position.setY(position.getY() + dy);
    position.setAngle(position.getAngle() + da);
}

void Robot::setPos(double x, double y, double a) {
    position.setX(x);
    position.setY(y);
    position.setAngle(a);
}

void Robot::setEncoder(int32_t left, int32_t right) {
    previous_encoder_left = left;
    previous_encoder_right = right;
    encoderLeft->write(left);
    encoderRight->write(right);
}

void Robot::setTrackMm(double trackMm) {
    track_mm = trackMm;
    if (sd_present) {
        jsonData["track_mm"] = trackMm;
        File file = SD.open("encoder.json", FILE_WRITE_BEGIN);
        serializeJson(jsonData, file);
        file.close();
    }
}

void Robot::setRightWheelDiam(double rightWheelDiam) {
    right_wheel_diam = rightWheelDiam;
    if (sd_present) {
        jsonData["right_wheel_diam"] = right_wheel_diam;
        File file = SD.open("encoder.json", FILE_WRITE_BEGIN);
        serializeJson(jsonData, file);
        file.close();
    }
}

void Robot::setLeftWheelDiam(double leftWheelDiam) {
    left_wheel_diam = leftWheelDiam;
    if (sd_present) {
        jsonData["left_wheel_diam"] = left_wheel_diam;
        File file = SD.open("encoder.json", FILE_WRITE_BEGIN);
        serializeJson(jsonData, file);
        file.close();
    }
}

std::vector<uint8_t> Robot::getRawData() {
    size_t size = (3 + 2) * sizeof(double);
    auto v = std::vector<uint8_t>(size);
    uint8_t *data = v.data();
    *((double *) data) = position.getX();
    *((double *) (data + sizeof(double))) = position.getY();
    *((double *) (data + 2 * sizeof(double))) = position.getAngle();
    *((double *) (data + 3 * sizeof(double))) = evaluatedLeft;
    *((double *) (data + 4 * sizeof(double))) = evaluatedRight;
    return v;
}

void Robot::calibration_went_forward(double distance) {
    int32_t eLeft = encoderLeft->read();
    int32_t eRight = encoderRight->read();

    int32_t left = eLeft - calibrationLeft;
    int32_t right = eRight - calibrationRight;
    double left_d = left * left_wheel_diam;
    double right_d = right * right_wheel_diam;
    if(left_d < 0){
        left_d *= -1;
        left_wheel_diam *= -1;
    }
    if(right_d < 0){
        right_d *= -1;
        right_wheel_diam *= -1;
    }
    double corr = left_d/right_d;
    right_wheel_diam *= corr;
    right_d *= corr;
    double c_distance = (left_d+right_d)/2;
    corr = distance/c_distance;
    left_wheel_diam *= corr;
    right_wheel_diam *= corr;
}

void Robot::calibration_begin() {
    calibrationLeft = encoderLeft->read();;
    calibrationRight = encoderRight->read();;
}
