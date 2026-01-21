/*******************************************************************************
 *				 _ _                                             _ _
				|   |                                           (_ _)
				|   |        _ _     _ _   _ _ _ _ _ _ _ _ _ _   _ _
				|   |       |   |   |   | |    _ _     _ _    | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |_ _ _  |   |_ _|   | |   |   |   |   |   | |   |
				|_ _ _ _ _| |_ _ _ _ _ _| |_ _|   |_ _|   |_ _| |_ _|
								(C)2022 Lumi
 * Copyright (c) 2022
 * Lumi, JSC.
 * All Rights Reserved
 *
 * File name: uart_service.cpp
 *
 * Description:
 *
 *
 * Last Changed By:  $Author: duongnv $
 * Revision:         $Revision: 1.0.0 $
 * Last Changed:     $Date: January 21, 2026 $
 *
 * Code sample:
 ******************************************************************************/
#include "uart_service.h"
#include <Arduino.h>
#include <HardwareSerial.h>

UartService::UartService(HardwareSerial *serialPort, const char *tag,
                         uint32_t baudRate)
    : _serial(serialPort), _tag(tag), _baudRate(baudRate) {}

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
    Serial1.print("[");
    Serial1.print(_tag);
    Serial1.print("] UART initialized at ");
    Serial1.print(_baudRate);
    Serial1.println(" baud");
  }
}

void UartService::logError(const char *message) {
  if (_tag && message && _serial != NULL) {
    _serial->print("[");
    _serial->print(_tag);
    _serial->print("] ERROR: ");
    _serial->println(message);
  }
}

void UartService::logInfo(const char *message) {
  if (_tag && message && _serial != NULL) {
    _serial->print("[");
    _serial->print(_tag);
    _serial->print("] INFO: ");
    _serial->println(message);
  }
}
