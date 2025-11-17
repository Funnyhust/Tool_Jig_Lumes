#ifndef ZERO_DETECT_H
#define ZERO_DETECT_H

#include <Arduino.h>

// Định nghĩa pin cho zero detect (giống cách dùng PC13 trong main.cpp)
#define ZERO_DETECT_PORT_1 PC0
#define ZERO_DETECT_PORT_2 PC1
#define ZERO_DETECT_PORT_3 PC2
#define ZERO_DETECT_PORT_4 PC3


void zero_detect_init(void);
void zero_detect_process(void);
void zero_detect_get_result(uint32_t* counts);  // Trả về count cho 4 kênh (counts phải có ít nhất 4 phần tử)

#endif