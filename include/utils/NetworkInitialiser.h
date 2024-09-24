//
// Created by fogoz on 24/06/2024.
//

#ifndef TEENSYCODE_NETWORKINITIALISER_H
#define TEENSYCODE_NETWORKINITIALISER_H

#include "ArduinoJson.h"
#include "SD.h"
#include "QNEthernet.h"

namespace utils::network {
    bool initialiseNetwork();
}
#endif //TEENSYCODE_NETWORKINITIALISER_H
