#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// UART Configuration
// ============================================================================
// UART1 (PA9/PA10)  → BL0906 Channel 1
// UART2 (PA2/PA3)   → BL0906 Channel 2
// UART3 (PB10/PB11) → BL0906 Channel 3
// UART4 (PC10/PC11) → BL0906 Channel 4
// UART5 (PC12/PD2)  → Debug
#define UART_DEBUG Serial5
#define UART_BL0906_1 Serial1
#define UART_BL0906_2 Serial2
#define UART_BL0906_3 Serial3
#define UART_BL0906_4 Serial4

#define POWER_CONTROL_SIGNAL_PIN PC6
#define POWER_CONTROL_PIN_1 PD15
#define POWER_CONTROL_PIN_2 PD14
#define POWER_CONTROL_PIN_3 PD13
#define POWER_CONTROL_PIN_4 PD12

#define SCL_PIN_1 PB12
#define SDA_PIN_1 PB13
#define SCL_PIN_2 PA6
#define SDA_PIN_2 PA7
#define SCL_PIN_3 PE11
#define SDA_PIN_3 PE10
#define SCL_PIN_4 PB6
#define SDA_PIN_4 PB7

#define ZERO_DETECT_PORT_1 PB14
#define ZERO_DETECT_PORT_2 PC4
#define ZERO_DETECT_PORT_3 PE12
#define ZERO_DETECT_PORT_4 PB8

#define CON_RELAY1A PD9  // Relay 1A
#define CON_RELAY1B PD8  // Relay 1B
#define CON_RELAY1C PB15 // Relay 1C
#define CON_RELAY2A PB1  // Relay 2A
#define CON_RELAY2B PB0  // Relay 2B
#define CON_RELAY2C PC5  // Relay 2C
#define CON_RELAY3A PE15 // Relay 3A
#define CON_RELAY3B PE14 // Relay 3B
#define CON_RELAY3C PE13 // Relay 3C
#define CON_RELAY4A PE1  // Relay 4A
#define CON_RELAY4B PE0  // Relay 4B
#define CON_RELAY4C PB9  // Relay 4C

#define PROCESS_DEBUG_ENABLE true
#define BL0906_DBG_EN false
#define EEPROM_TEST_ENABLE true
#endif