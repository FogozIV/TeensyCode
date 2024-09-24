//
// Created by fogoz on 26/06/2024.
//

#include "utils/TimeUtils.h"

namespace utils::time{
    uint64_t micros64() {
        static uint32_t lastTime = 0;
        static uint32_t loopTime = 0;
        uint32_t now = micros();
        if(now < lastTime){
            loopTime++;
        }
        lastTime = now;
        return ((uint64_t)loopTime << 32) + now;
    }
}

