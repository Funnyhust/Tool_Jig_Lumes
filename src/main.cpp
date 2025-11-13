#include <Arduino.h>
#include "services/relay/relay_service.h"

RelayService relayService;

void setup() {
    Serial.begin(9600);
    
    relayService.init();
    
    Serial.println("Relay system initialized");
}

void loop() {
    // Ví dụ sử dụng:
    // relayService.turnOn(RELAY_1A);
    // relayService.turnOff(RELAY_1B);
    // relayService.toggle(RELAY_2A);
    // relayService.setRelayIndex(RELAY_3C, 1);
    relayService.testFullRelay();
    delay(1000);
}