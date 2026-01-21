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
 * File name: uart_service.h
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
#ifndef UART_SERVICE_H
#define UART_SERVICE_H

#include <Arduino.h>
#include <HardwareSerial.h>

class UartService {
public:
  UartService(HardwareSerial *serialPort, const char *tag,
              uint32_t baudRate = 9600);
  void begin(int8_t rxPin = -1, int8_t txPin = -1);
  HardwareSerial *getSerial() const { return _serial; }
  void logError(const char *message);
  void logInfo(const char *message);

private:
  HardwareSerial *_serial;
  const char *_tag;
  uint32_t _baudRate;
};

#endif // UART_SERVICE_H
