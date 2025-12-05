#include <Arduino.h>
#include <HardwareSerial.h>
#include "services/relay/relay_service.h"
#include "app/process.h"
#include "services/zero_detect/zero_detect.h"
#include "services/eeprom_at24c02/at24c02.h"
#include "config.h"

RelayService relayService;

// PC13 là LED tích hợp trên STM32F103VE
#define LED_PIN PC13

// Biến để điều khiển blink LED
volatile bool ledBlinkEnable = false;
volatile bool ledState = false;
volatile uint32_t ledLastBlinkTime = 0;

// Hàm delay có blink LED
void delayWithBlink(uint32_t ms) {
    uint32_t startTime = millis();
    while (millis() - startTime < ms) {
        if (ledBlinkEnable) {
            uint32_t currentTime = millis();
            if (currentTime - ledLastBlinkTime >= 200) {  // Blink mỗi 200ms
                ledState = !ledState;
                digitalWrite(LED_PIN, ledState ? HIGH : LOW);
                ledLastBlinkTime = currentTime;
            }
        }
        delay(10);  // Delay nhỏ để không chiếm CPU
    }
}

void setup() {
    // Khởi tạo LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    relayService.init();
    process_init();
    ledBlinkEnable = true;
    start_process();
    ledBlinkEnable = false;
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    UART_DEBUG.println("Loop test read EEPROM");
    at24c02_test(1);
    delay(3000);
    // Blink LED trong lúc chờ giữa các lần test

}