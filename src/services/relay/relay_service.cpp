#include "relay_service.h"

// Map relay với GPIO port và pin (theo Middle/relay.c)
// Port: 0=PA, 1=PB, 2=PC, 3=PD, 4=PE
// Pin: 0-15 (số pin trong port)
// {GPIOE, GPIO_PIN_1} -> port=4, pin=1
// {GPIOE, GPIO_PIN_0} -> port=4, pin=0
// {GPIOB, GPIO_PIN_9} -> port=1, pin=9
// {GPIOD, GPIO_PIN_3} -> port=3, pin=3
// {GPIOD, GPIO_PIN_4} -> port=3, pin=4
// {GPIOD, GPIO_PIN_5} -> port=3, pin=5
// {GPIOD, GPIO_PIN_6} -> port=3, pin=6
// {GPIOD, GPIO_PIN_7} -> port=3, pin=7
// {GPIOB, GPIO_PIN_3} -> port=1, pin=3
// {GPIOB, GPIO_PIN_4} -> port=1, pin=4
// {GPIOB, GPIO_PIN_5} -> port=1, pin=5
// {GPIOB, GPIO_PIN_6} -> port=1, pin=6
const RelayMap_t RelayService::_relayMap[NUM_RELAYS] = {
    {4, 1},  // RELAY_1A - PE1
    {4, 0},  // RELAY_1B - PE0
    {1, 9},  // RELAY_1C - PB9
    {3, 3},  // RELAY_2A - PD3
    {3, 4},  // RELAY_2B - PD4
    {3, 5},  // RELAY_2C - PD5
    {3, 6},  // RELAY_3A - PD6
    {3, 7},  // RELAY_3B - PD7
    {1, 3},  // RELAY_3C - PB3
    {1, 4},  // RELAY_4A - PB4
    {1, 5},  // RELAY_4B - PB5
    {1, 6}   // RELAY_4C - PB6
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

