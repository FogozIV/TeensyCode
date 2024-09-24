//
// Created by fogoz on 24/09/2024.
//

#ifndef TEENSYCODE_PINMODEEX_H
#define TEENSYCODE_PINMODEEX_H
#include <initializer_list>
#include "Arduino.h"

inline void pinMode(std::initializer_list<uint8_t> pins, uint8_t mode)
{
    for (uint8_t pin : pins)
    {
        pinMode(pin, mode);
    }
}
#endif //TEENSYCODE_PINMODEEX_H
