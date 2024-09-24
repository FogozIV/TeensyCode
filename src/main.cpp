#include <Arduino.h>
#include "packet/Import.h"
#include "utils/NetworkInitialiser.h"
#include <MQTTPubSubClient.h>
#include <sstream>
#include "chrono"
#include "Robot.h"

#define MQTT_CLIENT "masterrp.local"
#define MQTT_PORT 1883
/*
    khoih-prog/AsyncUDP_Teensy41@^1.2.1
	khoih-prog/Teensy41_AsyncTCP@^1.1.0
 */
using namespace std::chrono;

qindesign::network::EthernetClient client;
MQTTPubSubClient mqttClient;
//std::mutex networkMutex;

uint64_t counter = 0;
system_clock::time_point looping_time;
Robot* robot;
void setup(){
    Serial.begin(256000);
    //threads.setSliceMicros(100);
    Serial.println("Hello initialising teensy 4.1");
    utils::network::initialiseNetwork();
    Serial.println("Network initialised");
    Serial.print("IPAdress : ");
    client.connect(MQTT_CLIENT, MQTT_PORT);
    Serial.println(client.localIP());
    Serial.print("Connection to mqtt host : ");
    Serial.println(client.connected());
    mqttClient.setCleanSession(true);
    mqttClient.begin(client);
    Serial.print("Is MQTT ok : ");
    mqttClient.connect(qindesign::network::MDNS.hostname());
    Serial.println(mqttClient.isConnected());
    looping_time = system_clock::now();
    robot = new Robot(0,1,2,3);
    mqttClient.subscribe("position_set", [](const String& str, size_t size){
        const char* data = str.begin();
        size_t data_size = sizeof (double);
        robot->setPos(*((double*)data), *((double*)(data+data_size)), *((double*)(data+2*data_size)));
    });
    mqttClient.subscribe("position_set_string", [](const String& str, size_t size){
        std::stringstream ss(str.c_str());
        double x, y, a;
        ss >> x;
        ss >> y;
        ss >> a;
        robot->setPos(x, y, a);
    });
    mqttClient.subscribe("encoder_set", [](const String& str, size_t size){
        const char* data = str.begin();
        size_t data_size = sizeof (int32_t);
        robot->setEncoder(*((int32_t*)data), *((int32_t*)(data + data_size)));
    });
    mqttClient.subscribe("encoder_set_string", [](const String& str, size_t size){
       int32_t left, right;
       std::stringstream ss(str.c_str());
       ss >> left;
       ss >> right;
       robot->setEncoder(left, right);
    });
    mqttClient.subscribe("encoder_left_set", [](const String& str, size_t size){
       const char* data = str.begin();
       robot->setLeftWheelDiam(*((double*)data));
    });
    mqttClient.subscribe("encoder_left_set_string", [](const String& str, size_t size){
        double left_wheel_diam;
        std::stringstream ss(str.c_str());
        ss >> left_wheel_diam;
        robot->setLeftWheelDiam(left_wheel_diam);
    });
    mqttClient.subscribe("encoder_right_set", [](const String& str, size_t size){
        const char* data = str.begin();
        robot->setRightWheelDiam(*((double*)data));
    });
    mqttClient.subscribe("encoder_right_set_string", [](const String& str, size_t size){
        double right_wheel_diam;
        std::stringstream ss(str.c_str());
        ss >> right_wheel_diam;
        robot->setRightWheelDiam(right_wheel_diam);
    });
    mqttClient.subscribe("encoder_track_set", [](const String& str, size_t size){
        const char* data = str.begin();
        robot->setTrackMm(*((double*)data));
    });
    mqttClient.subscribe("encoder_track_set_string", [](const String& str, size_t size){
        double track_mm;
        std::stringstream ss(str.c_str());
        ss >> track_mm;
        robot->setTrackMm(track_mm);
    });
    /*mqttClient.subscribe("test/topic", [](const String& data, size_t size){
        Serial.println(data);
    });*/
}
void loop(){
    //networkMutex.lock();
    mqttClient.update();
    //networkMutex.unlock();
    if(looping_time + 5ms >= system_clock::now()){
        looping_time += 5ms;
        robot->update();
        auto data = robot->getRawData();
        mqttClient.publish("position_data", std::get<0>(data), std::get<1>(data), false, 2);
    }
    /*
    if(connected){
        if(millis() - previous_time >= 5){
            uint32_t a = micros();
            networkMutex.lock();
            mqttClient.publish("test/topic", String(cos(double(millis())/100*2*PI)));
            networkMutex.unlock();
            Serial.println(micros() - a);
            counter++;
            previous_time = millis();
        }
    }else{
        Serial.println("RIP MQTT Client");
        networkMutex.lock();
        client.connect(MQTT_CLIENT, MQTT_PORT);
        mqttClient.connect(qindesign::network::MDNS.hostname());
        networkMutex.unlock();
    }
     */


}