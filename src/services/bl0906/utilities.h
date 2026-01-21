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
 * File name: utilities.h
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
#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

// Macros
#define BIT(n) (1 << (n))
#define foreach(i, n) for (uint8_t i = 0; i < (n); i++)

// Time helper
static inline bool clock_time_exceed_ms(uint32_t start_time,
                                        uint32_t timeout_ms) {
  return (millis() - start_time) >= timeout_ms;
}

static inline uint32_t clock_time_ms(void) { return millis(); }

static inline void sleep_ms(uint32_t ms) { delay(ms); }

#endif // UTILITIES_H
