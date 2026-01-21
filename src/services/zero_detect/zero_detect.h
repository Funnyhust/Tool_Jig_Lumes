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
 * File name: zero_detect.h
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
#ifndef ZERO_DETECT_H
#define ZERO_DETECT_H

#include <Arduino.h>

// Định nghĩa pin cho zero detect (giống cách dùng PC13 trong main.cpp)
// #define ZERO_DETECT_PORT_1 PC0
// #define ZERO_DETECT_PORT_2 PC1
// #define ZERO_DETECT_PORT_3 PC2
// #define ZERO_DETECT_PORT_4 PC3

void zero_detect_init_pin(void);
void zero_detect_init(void);

void zero_detect_process(void);
bool zero_detect_get_result(uint8_t channel); // Trả về count cho 4 kênh (counts
                                              // phải có ít nhất 4 phần tử)

#endif