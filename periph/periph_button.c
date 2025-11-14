/*
 * periph_button.c
 *
 *  Created on: Jul 8, 2024
 *      Author: DungTranBK
 */

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "vendor/common/system_time.h"
#include "vendor/common/mesh_config.h"
#include "proj_lib/sig_mesh/app_mesh.h"
#include "vendor/common/mesh_node.h"
#include "../utilities.h"
#include "../config_board.h"
#include "periph_button.h"

#include "../debug.h"
#ifdef BTN_DBG_EN
#define DBG_BTN_SEND_STR(x)     Dbg_sendString((s8*)x)
#define DBG_BTN_SEND_INT(x)     Dbg_sendInt(x)
#define DBG_BTN_SEND_HEX(x)     Dbg_sendHex(x)
#define DBG_BTN_SEND_HEX_ONE(x) Dbg_sendHexOneByte(x);
#else
#define DBG_BTN_SEND_STR(x)
#define DBG_BTN_SEND_INT(x)
#define DBG_BTN_SEND_HEX(x)
#define DBG_BTN_SEND_HEX_ONE(x)
#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                          PRIVATE FUNCTIONS DECLERATION                     */
/******************************************************************************/

/******************************************************************************/
/*                           EXPORT FUNCTIONS                                 */
/******************************************************************************/

typeButton_HandleStateCallbackFunc pvButton_HandleStateCallback = NULL;

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

static u32 scan_interval_ms = BUTTON_NORMAL_SCAN_INTERVAL_MS;
static uint32_t opt_button_scan_timer[NUMBER_BUTTON] = {0};


#if NUMBER_SWITCH_BUTTON == 1
static u16 opt_btn_map_pin[NUMBER_BUTTON+1] = {
		 PIN_SC_BUTTON_0, PIN_SC_BUTTON_1, PIN_BUTTON_0, PIN_BUTTON_2
	};
#elif NUMBER_SWITCH_BUTTON == 2
static u16 opt_btn_map_pin[NUMBER_BUTTON] = {
		 PIN_SC_BUTTON_0, PIN_SC_BUTTON_1, PIN_BUTTON_0, PIN_BUTTON_2
	};
#elif NUMBER_SWITCH_BUTTON == 3
static u16 opt_btn_map_pin[NUMBER_BUTTON] = {
		 PIN_SC_BUTTON_0, PIN_SC_BUTTON_1, PIN_BUTTON_0, PIN_BUTTON_1, PIN_BUTTON_2
	};
#endif

opt_button_params_t  opt_button_params[NUMBER_BUTTON];

#define OPTION_BUTTON_PRESS_TIME_OUT 	    TIMER_700MS
#define OPTION_BUTTON_PRESS_ELAPSED_TIME(x) (clock_time_ms() - opt_button_params[x].press_last_t)
#define OPTION_BUTTON_HOLD_TIME(x)          clock_time_get_elapsed_time(opt_button_params[x].start_hold_t)

/******************************************************************************/
/*                       PRIVATE FUNCTION DECLERATION                         */
/******************************************************************************/
static void button_call_handle_function(uint8_t idx, u8 state);

/******************************************************************************/
/*                            EXPORT FUNCTION                                 */
/******************************************************************************/

#if NUMBER_SWITCH_BUTTON == 1
#define OPTION_BUTTON_STATE_PIN_STATE(x)   \
			((gpio_read(opt_btn_map_pin[x])& get_io_bit_from_io_pin(opt_btn_map_pin[x]))?1:0)
/**
 * @func    OPTION_BUTTON_STATE
 * @brief
 * @param   None
 * @retval  None
 */
u8 OPTION_BUTTON_STATE(u8 idx)
{
	if(idx < ELE_RELAY_OFFSET)
	{
		return OPTION_BUTTON_STATE_PIN_STATE(idx);
	}
	else {
		u8 b1 = OPTION_BUTTON_STATE_PIN_STATE(ELE_RELAY_OFFSET);
		u8 b2 = OPTION_BUTTON_STATE_PIN_STATE(ELE_RELAY_OFFSET+1);
		return (b1&b2);
	}
}

#else
#define OPTION_BUTTON_STATE(x)              ((gpio_read(opt_btn_map_pin[x])& get_io_bit_from_io_pin(opt_btn_map_pin[x]))?1:0)
#endif

/**
 * @func    option_button_init
 * @brief
 * @param   None
 * @retval  None
 */
void option_button_init(void)
{
	foreach(i, sizeof(opt_btn_map_pin))
	{
		gpio_set_func(opt_btn_map_pin[i], AS_GPIO);
		gpio_set_input_en(opt_btn_map_pin[i], true);
		gpio_set_output_en(opt_btn_map_pin[i], false);
		#if PCBA_8258_SEL == PCBA_8258_DONGLE_48PIN
		gpio_setup_up_down_resistor(opt_btn_map_pin[i], PM_PIN_PULLUP_10K);
		#endif
	}
	foreach(i, NUMBER_BUTTON)
	{
		opt_button_params[i].press_last_t = 0;
		opt_button_params[i].press_cnt = 0;
		opt_button_params[i].press_many_time_f = false;
		opt_button_params[i].start_hold_t = 0;
		opt_button_params[i].prev_state = OPTION_BUTTON_RELEASE;
		opt_button_params[i].hold_step = 0;
		opt_button_params[i].poll_cnt = 0;
	}
}

/**
 * @func   option_button_callback_init
 * @brief  Init function to handle when option button press/hold/release
 * @param  Function pointer
 * @retval None
 */
void option_button_callback_init(
		typeButton_HandleStateCallbackFunc handleOptionButtonStateCallbackInit)
{
	if(handleOptionButtonStateCallbackInit != NULL){
		pvButton_HandleStateCallback = handleOptionButtonStateCallbackInit;
	}
}

/**
 * @func    option_button_press
 * @brief
 * @param
 * @retval  None
 */
static void option_button_press(u8 idx)
{
	if(pvButton_HandleStateCallback != NULL){
		pvButton_HandleStateCallback(idx, START_PRESS);
	}
	// Scan for press many time button
	if(OPTION_BUTTON_PRESS_ELAPSED_TIME(idx) < OPTION_BUTTON_PRESS_TIME_OUT){
		opt_button_params[idx].press_cnt++;
	}
	else{
		opt_button_params[idx].press_cnt = 1;
	}
	opt_button_params[idx].press_last_t = clock_time_ms();
	opt_button_params[idx].press_many_time_f = true;
}

/**
 * @func   button_call_handle_function
 * @brief
 * @param  Byte: Option Button State
 * @retval None
 */
static void button_call_handle_function(u8 idx, u8 state)
{
	if(pvButton_HandleStateCallback != NULL){
		pvButton_HandleStateCallback(idx, state);
	}
}

/**
 * @func   option_button_press_times
 * @brief  Count option button press time
 * @param  None
 * @retval None
 */
static void option_button_press_times(u8 idx)
{
	if(opt_button_params[idx].press_many_time_f != false){
		if(OPTION_BUTTON_PRESS_ELAPSED_TIME(idx) > OPTION_BUTTON_PRESS_TIME_OUT){
			if(OPTION_BUTTON_PRESS_ELAPSED_TIME(idx) > OPTION_BUTTON_PRESS_TIME_OUT + TIMER_200MS){
				opt_button_params[idx].press_many_time_f = false;
				return;
			}
			switch (opt_button_params[idx].press_cnt)
			{
				case 1:
					button_call_handle_function(idx, PRESS_ONE_TIME);
					break;
				case 2:
					button_call_handle_function(idx, PRESS_TWO_TIME);
					break;
				case 3:
					button_call_handle_function(idx, PRESS_THREE_TIME);
					break;
				case 4:
					button_call_handle_function(idx, PRESS_FOUR_TIME);
					break;
				case 5:
					button_call_handle_function(idx, PRESS_FIVE_TIME);
					break;
				case 6:
					button_call_handle_function(idx, PRESS_SIX_TIME);
					break;
				case 8:
					button_call_handle_function(idx, PRESS_EIGHT_TIME);
					break;
				case 10:
					button_call_handle_function(idx, PRESS_TEN_TIME);
					break;
				case 12:
					button_call_handle_function(idx, PRESS_TWELVE_TIME);
					break;
				default:
					break;
			}
			opt_button_params[idx].press_many_time_f = false;
		}
	}
}

/**
 * @func   check_option_button_press
 * @brief  Return status of option button press
 * @param  None
 * @retval Byte: Status
 */
static u8 check_option_button_press(uint8_t idx)
{
	if (OPTION_BUTTON_STATE(idx) == OPTION_BUTTON_PRESS){
		u8 boundCnt = 0;
		foreach (i, 4)
		{
			__delay_ms(4);
			if (OPTION_BUTTON_STATE(idx) == OPTION_BUTTON_PRESS)
			{
				boundCnt++;
			}
		}
		if (boundCnt == 4){
			return true;
		}
	}
	return false;
}

/**
 * @func   option_button_scan
 * @brief  Option button scan function
 * @param  None
 * @retval None
 */
void option_button_scan(void)
{
	foreach(i, NUMBER_BUTTON)
	{
		scan_interval_ms = (opt_button_params[i].prev_state  \
				== OPTION_BUTTON_PRESS)?BUTTON_FAST_SCAN_INTERVAL_MS:BUTTON_NORMAL_SCAN_INTERVAL_MS;
		if(clock_time_get_elapsed_time(opt_button_scan_timer[i]) >= scan_interval_ms)
		{
			if((opt_button_params[i].prev_state == OPTION_BUTTON_RELEASE) && (OPTION_BUTTON_STATE(i) == OPTION_BUTTON_PRESS))
			{
				if(check_option_button_press(i) == true)
				{
					option_button_press(i);
					opt_button_params[i].poll_cnt = 0;
				}
			}
			else if((opt_button_params[i].prev_state == OPTION_BUTTON_PRESS)
					&& (OPTION_BUTTON_STATE(i) == OPTION_BUTTON_PRESS))
			{
				/*
				DBG_BTN_SEND_STR("\r\nOption Button Hold: ");
				DBG_BTN_SEND_INT(i);
				*/
				opt_button_params[i].poll_cnt++;
				if(opt_button_params[i].hold_step == 0xFF)
				{
					opt_button_params[i].start_hold_t = clock_time_ms();
					opt_button_params[i].hold_step = 0;
				}
				if(OPTION_BUTTON_HOLD_TIME(i) > OPTION_BUTTON_ERROR_HOLD_TIME){
					opt_button_params[i].hold_step = 0xFE;
				}

				switch(opt_button_params[i].hold_step)
				{
				case 0:
					if(opt_button_params[i].poll_cnt >= MIN_POLL_COUNTER_TO_CHANGE_HOLD_STEP)
					{
						opt_button_params[i].hold_step = 1;
					}
					break;

				case 1:
					if(OPTION_BUTTON_HOLD_TIME(i) > TIMER_50MS)
					{
						opt_button_params[i].hold_step = 2;
						button_call_handle_function(i, HOLD_50MS);
					}
					break;

				case 2:
					if(OPTION_BUTTON_HOLD_TIME(i) > TIMER_500MS)
					{
						opt_button_params[i].hold_step = 3;
						button_call_handle_function(i, HOLD_500MS);
					}
					break;

				case 3:
					if(OPTION_BUTTON_HOLD_TIME(i) > TIMER_800MS)
					{
						opt_button_params[i].hold_step = 4;
						button_call_handle_function(i, HOLD_2S);
					}
					break;
				case 4:
					if(OPTION_BUTTON_HOLD_TIME(i) > TIMER_5S)
					{
						opt_button_params[i].hold_step = 5;
						button_call_handle_function(i, HOLD_5S);
					}
					break;
				case 5:
					if(OPTION_BUTTON_HOLD_TIME(i) > TIMER_10S)
					{
						opt_button_params[i].hold_step = 6;
						button_call_handle_function(i, HOLD_10S);
					}
					break;

				case 6:
					if(OPTION_BUTTON_HOLD_TIME(i) > TIMER_15S)
					{
						opt_button_params[i].hold_step = 7;
						button_call_handle_function(i, HOLD_15S);
					}
					break;
				}
			}
			else if((opt_button_params[i].prev_state == OPTION_BUTTON_PRESS)
					&&(OPTION_BUTTON_STATE(i) == OPTION_BUTTON_RELEASE))
			{
				opt_button_params[i].hold_step = 0xFF;
				button_call_handle_function(i, RELEASE);
			}
			else if((opt_button_params[i].prev_state == OPTION_BUTTON_RELEASE)
					&&(OPTION_BUTTON_STATE(i) == OPTION_BUTTON_RELEASE))
			{
				option_button_press_times(i);
				if(opt_button_params[i].hold_step < 0xFF){
					// DBG_BTN_SEND_STR("\n RELEASE");
					button_call_handle_function(i, RELEASE);
				}
				else
				{
					button_call_handle_function(i, NO_PRESS);
				}
				opt_button_params[i].hold_step = 0xFF;
			}
			opt_button_params[i].prev_state = OPTION_BUTTON_STATE(i);
			opt_button_scan_timer[i] = clock_time_ms();
		}
	}
}
