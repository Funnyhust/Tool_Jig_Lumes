/*
 * periph_button.h
 *
 *  Created on: Jul 8, 2024
 *      Author: DungTranBK
 */

#ifndef PERIPH_BUTTON_H_
#define PERIPH_BUTTON_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "tl_common.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
typedef struct
{
	u32  press_last_t;
	u8   press_cnt;
	BOOL press_many_time_f;

	u32  start_hold_t;
	u8   prev_state;
	u8   hold_step;
	u8   poll_cnt;
}opt_button_params_t;


typedef void (*typeButton_HandleStateCallbackFunc)(u8, u8);

#define OPTION_BUTTON_PRESS         0x00
#define OPTION_BUTTON_RELEASE       0x01

#define MIN_POLL_COUNTER_TO_CHANGE_HOLD_STEP  10
#define OPTION_BUTTON_ERROR_HOLD_TIME         TIMER_20S


#define BUTTON_FAST_SCAN_INTERVAL_MS      TIMER_10MS
#define BUTTON_NORMAL_SCAN_INTERVAL_MS    TIMER_50MS

/******************************************************************************/
/*                              EXPORT FUNCTION                               */
/******************************************************************************/
void option_button_init(void);
void option_button_callback_init(typeButton_HandleStateCallbackFunc handleOptionButtonStateCallbackInit);
void option_button_scan(void);

#endif /* PERIPH_BUTTON_H_ */
