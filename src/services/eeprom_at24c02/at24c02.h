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
 * File name: at24c02.h
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
#ifndef AT24C02_H
#define AT24C02_H

#include <Arduino.h>

#define AT24C02_ADDRESS 0x50

// Đảm bảo các hàm C này có thể link được từ C++
#ifdef __cplusplus
extern "C" {
#endif

void at24c02_init(uint8_t bus_num);
bool at24c02_write(uint8_t address, uint8_t data);
bool at24c02_write_block(uint8_t address, uint8_t *data, uint8_t length);
uint8_t at24c02_read(uint8_t address);
bool at24c02_read_block(uint8_t address, uint8_t *data, uint8_t length);
void at24c02_test(uint8_t bus_num);

#ifdef __cplusplus
}
#endif

#endif