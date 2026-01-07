#include "relay_service.h"

// Map relay với GPIO port và pin theo sơ đồ phần cứng
// Port: 0=PA, 1=PB, 2=PC, 3=PD, 4=PE
// Pin: 0-15 (số pin trong port)
const RelayMap_t RelayService::_relayMap[NUM_RELAYS] = {
    {3, 8},   // RELAY_1A - PD8
    {3, 9},   // RELAY_1B - PD9
    {1, 15},  // RELAY_1C - PB15
    {1, 1},   // RELAY_2A - PB1
    {1, 0},   // RELAY_2B - PB0
    {2, 5},   // RELAY_2C - PC5
    {4, 15},  // RELAY_3A - PE15
    {4, 14},  // RELAY_3B - PE14
    {4, 13},  // RELAY_3C - PE13
    {4, 1},   // RELAY_4A - PE1
    {4, 0},   // RELAY_4B - PE0
    {1, 9}    // RELAY_4C - PB9
};

void RelayService::init() {
    for (uint8_t i = 0; i < NUM_RELAYS; i++) {
        uint8_t arduinoPin = GPIO_TO_ARDUINO_PIN(_relayMap[i].port, _relayMap[i].pin);
        pinMode(arduinoPin, OUTPUT);
        digitalWrite(arduinoPin, LOW);
        _relayStates[i] = false;
    }
}


void RelayService::setRelayIndex(uint8_t idRelay, uint8_t setRelay) {
    if (idRelay >= NUM_RELAYS) return;
    uint8_t arduinoPin = GPIO_TO_ARDUINO_PIN(_relayMap[idRelay].port, _relayMap[idRelay].pin);
    digitalWrite(arduinoPin, setRelay ? HIGH : LOW);
    _relayStates[idRelay] = (setRelay != 0);
}



void RelayService::setRelayState(uint8_t idRelay, bool state) {
    if (idRelay >= NUM_RELAYS) return;
    if(state){
        setRelayIndex(idRelay, 1);
    }else{
        setRelayIndex(idRelay, 0);
    }
}

void RelayService::turnOn(uint8_t idRelay) {
    setRelayIndex(idRelay, 1);
}


void RelayService::turnOff(uint8_t idRelay) {
    setRelayIndex(idRelay, 0);
}


void RelayService::toggle(uint8_t idRelay) {
    if (idRelay >= NUM_RELAYS) return;
    setRelayIndex(idRelay, !_relayStates[idRelay]);
}


bool RelayService::isOn(uint8_t idRelay) const {
    if (idRelay >= NUM_RELAYS) return false;
    return _relayStates[idRelay];
    }

void RelayService::turnOnAll() {
    for (uint8_t i = 0; i < NUM_RELAYS; i++) {
        turnOn(i);
    }
}

void RelayService::turnOffAll() {
    for (uint8_t i = 0; i < NUM_RELAYS; i++) {
        turnOff(i);
    }
}


void RelayService::testFullRelay() {
    turnOnAll();
    delay(1000);
    turnOffAll();
    delay(1000);
}

