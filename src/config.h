#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>


// Serial1 → Debug
// Serial2, Serial3, Serial4, Serial5 → 4 kênh BL0906 (channel 0, 1, 2, 3)
#define UART_DEBUG    Serial1
#define UART_BL0906_1 Serial2
#define UART_BL0906_2 Serial3
#define UART_BL0906_3 Serial4
#define UART_BL0906_4 Serial5


#endif