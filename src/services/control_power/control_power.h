#ifndef CONTROL_POWER_H
#define CONTROL_POWER_H

#include <string>
#include "config.h"


void control_power_init(void);

bool is_return_power_control_signal();

void control_power_shutdown(void);

void control_power_on(void);


#endif