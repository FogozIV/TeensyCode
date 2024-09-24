//
// Created by fogoz on 24/06/2024.
//
#include "utils/NetworkInitialiser.h"

#ifndef SELECTED_CHIP
#define SELECTED_CHIP BUILTIN_SDCARD
#endif

namespace utils::network{
    bool initialiseNetwork(){
        bool sd_ok = true;
        if(!SD.begin(SELECTED_CHIP)) {
            Serial.println("Card failed or not present");
            sd_ok = false;
        }
        if(sd_ok){
            File config = SD.open("/config.json");
            JsonDocument jsonConfig;
            deserializeJson(jsonConfig,config);
            if(jsonConfig["mac"].is<JsonArray>()){
                uint8_t macAddress[] = {jsonConfig["mac"][0], jsonConfig["mac"][1], jsonConfig["mac"][2], jsonConfig["mac"][3], jsonConfig["mac"][4], jsonConfig["mac"][5]};
                qindesign::network::Ethernet.setMACAddress(macAddress);
                Serial.println("Using mac address in the json.");
            }
            if(jsonConfig["ip"].is<const char*>() && jsonConfig["mask"].is<const char*>() && jsonConfig["gateway"].is<const char*>()){
                IPAddress ip;
                ip.fromString(jsonConfig["ip"].as<const char*>());
                Serial.print("Using ip : ");
                Serial.println(ip);
                IPAddress mask;
                mask.fromString(jsonConfig["mask"].as<const char*>());
                Serial.print("Using mask : ");
                Serial.println(mask);
                IPAddress gateway;
                gateway.fromString(jsonConfig["gateway"].as<const char*>());
                Serial.print("Using gateway : ");
                Serial.println(gateway);
                qindesign::network::Ethernet.begin(ip, mask, gateway);
            }else{
                qindesign::network::Ethernet.begin();
            }
            if(jsonConfig["hostname"].is<const char*>()){
                qindesign::network::Ethernet.setHostname(jsonConfig["hostname"]);
                qindesign::network::MDNS.begin(jsonConfig["hostname"]);
                Serial.print("Using hostname : ");
                Serial.println(qindesign::network::Ethernet.hostname());
            }else{
                qindesign::network::Ethernet.setHostname("teensy");
                qindesign::network::MDNS.begin("teensy");
            }
        }else{
            qindesign::network::Ethernet.begin();
            qindesign::network::Ethernet.setHostname("teensy");
            qindesign::network::MDNS.begin("teensy");
        }
        return sd_ok;
    }
}
