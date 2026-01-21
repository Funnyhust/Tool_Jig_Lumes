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
 * File name: process.h
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
#ifndef PROCESS_H
#define PROCESS_H

#include <Arduino.h>

// Forward declaration
class RelayService;

// Biến để điều khiển blink LED từ process
extern volatile bool ledBlinkEnable;

// Ngưỡng mặc định dùng để tính hệ số calibration
// Đơn vị:
// - Điện áp: mV
// - Dòng điện: mA
// - Công suất: mW
extern const uint32_t VOLTAGE_THRESHOLD[4];    // mV
extern const uint32_t CURRENT_THRESHOLD[4][3]; // mA
extern const uint32_t POWER_THRESHOLD[4][3];   // mW

extern bool zero_detect_result[4]; // Kết quả zero detect cho 4 kênh

// Khai báo hàm khởi tạo
void uart_init(void);
void process_init(void);
void start_process(void);
bool is_fail_once();
void debug_init(void);

#endif