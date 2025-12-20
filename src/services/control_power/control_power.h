#ifndef CONTROL_POWER_H
#define CONTROL_POWER_H

#include <string>
#include "config.h"

#define POWER_CONTROL_SIGNAL_PIN  PE12  // Chân đọc tín hiệu điều khiển nguồn - M4.3
#define POWER_CONTROL_PIN  PE11  // Xuất tín hiệu điều khiển relay bật nguồn - (M4.2)

void control_power_init(void);

bool is_return_power_control_signal();

void control_power_shutdown(void);

void control_power_on(void);


#endif