#include "uart_service.h"

UartService::UartService(HardwareSerial* serialPort, const char* tag, uint32_t baudRate)
    : _serial(serialPort), _tag(tag), _baudRate(baudRate) {
}

void UartService::begin(int8_t rxPin, int8_t txPin) {
    if (rxPin >= 0 && txPin >= 0) {
        #ifdef ESP32
        _serial->begin(_baudRate, SERIAL_8N1, rxPin, txPin);
        #else
        _serial->begin(_baudRate);
        #endif
    } else {
        _serial->begin(_baudRate);
    }
    _serial->setTimeout(1000);
    
    if (_tag) {
        Serial.print("[");
        Serial.print(_tag);
        Serial.print("] UART initialized at ");
        Serial.print(_baudRate);
        Serial.println(" baud");
    }
}

void UartService::logError(const char* message) {
    if (_tag && message) {
        Serial.print("[");
        Serial.print(_tag);
        Serial.print("] ERROR: ");
        Serial.println(message);
    }
}

void UartService::logInfo(const char* message) {
    if (_tag && message) {
        Serial.print("[");
        Serial.print(_tag);
        Serial.print("] INFO: ");
        Serial.println(message);
    }
}

