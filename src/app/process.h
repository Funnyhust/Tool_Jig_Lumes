#ifndef PROCESS_H
#define PROCESS_H

#include <Arduino.h>

// Forward declaration
class RelayService;

// Biến để điều khiển blink LED từ process
extern volatile bool ledBlinkEnable;

#define VOLTAGE_THRESHOLD 230
#define CURRENT_THRESHOLD 10
#define ACTIVE_POWER_THRESHOLD 10

#define VOLTAGE_THRESHOLD_LOW 215
#define CURRENT_THRESHOLD_LOW 196
#define ACTIVE_POWER_THRESHOLD_LOW 43

#define VOLTAGE_THRESHOLD_HIGH 240
#define CURRENT_THRESHOLD_HIGH 199
#define ACTIVE_POWER_THRESHOLD_HIGH 45

// Khai báo hàm khởi tạo
void process_init(void);
void start_process(void);

// typedef struct {
//     uint16_t voltage;
//     uint16_t current_1;
//     uint16_t current_2;
//     uint16_t current_3;
//     bool voltage_ok=false;
//     bool current_1_ok=false;
//     bool current_2_ok=false;
//     bool current_3_ok=false;
// } measurement_value_t;

#endif