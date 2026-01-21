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
 * File name: control_power.h
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
#ifndef CONTROL_POWER_H
#define CONTROL_POWER_H

#include "config.h"
#include <string>

void control_power_init(void);

bool is_return_power_control_signal();

void control_power_shutdown(void);

void control_power_on(void);

#endif