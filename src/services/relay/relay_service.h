#ifndef RELAY_SERVICE_H
#define RELAY_SERVICE_H

#include <Arduino.h>

enum {
    RELAY_1A,
    RELAY_1B,
    RELAY_1C,
    RELAY_2A,
    RELAY_2B,
    RELAY_2C,
    RELAY_3A,
    RELAY_3B,
    RELAY_3C,
    RELAY_4A,
    RELAY_4B,
    RELAY_4C,
    NUM_RELAYS
};

typedef struct {
    uint8_t port;  // GPIO port: 0=PA, 1=PB, 2=PC, 3=PD, 4=PE
    uint8_t pin;   // Pin number trong port (0-15)
} RelayMap_t;

// Helper macro để convert GPIO port và pin sang Arduino pin number
// STM32F103VE: PA=0-15, PB=16-31, PC=32-47, PD=48-63, PE=64-79
#define GPIO_TO_ARDUINO_PIN(port, pin) ((port) * 16 + (pin))

class RelayService {
public:
    void init();
    void setRelayIndex(uint8_t idRelay, uint8_t setRelay);
    void turnOn(uint8_t idRelay);
    void turnOff(uint8_t idRelay);
    void toggle(uint8_t idRelay);
    bool isOn(uint8_t idRelay) const;
    void turnOnAll();
    void turnOffAll();
    void testFullRelay();

private:
    static const RelayMap_t _relayMap[NUM_RELAYS];
    bool _relayStates[NUM_RELAYS];
};

#endif // RELAY_SERVICE_H
