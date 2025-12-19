#ifndef CONTROL_POWER_H
#define CONTROL_POWER_H

#include <string>
#include "config.h"

#define POWER_CONTROL_SIGNAL_PIN  A5  // Chân đọc tín hiệu điều khiển nguồn - M1.2
#define POWER_CONTROL_PIN  A4  // Xuất tín hiệu điều khiển relay bật nguồn - (M1.1)

void control_power_init(void);

bool is_return_power_control_signal();

void control_power_shutdown(void);

void control_power_on(void);


#endif