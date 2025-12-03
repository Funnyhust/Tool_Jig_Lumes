/*
 * bl0906.cpp
 *
 *  Created on:  Nov 12, 2025
 *      Author: DungTranBK
 */

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bl0906.h"
#include "utilities.h"
#include "../uart/uart_service.h"
#include "../../config.h"
// Debug macros - in ra Serial5
//#define BL0906_DBG_EN false
#ifdef  BL0906_DBG_EN
	#define DBG_BL0906_SEND_STR(x)          do { if (Serial) UART_DEBUG.print(x); } while(0)
	#define DBG_BL0906_SEND_STR_INFO(x)     do { if (Serial) UART_DEBUG.print(x); } while(0)
	#define DBG_BL0906_SEND_STR_ERROR(x)    do { if (Serial) {UART_DEBUG.print("[ERROR]");   UART_DEBUG.print(x);} } while(0)
	#define DBG_BL0906_SEND_INT(x)     do { if (Serial) UART_DEBUG.print(x); } while(0)
	#define DBG_BL0906_SEND_HEX(x)     do { if (Serial) { UART_DEBUG.print("0x"); UART_DEBUG.print(x, HEX); } } while(0)
	#define DBG_BL0906_SEND_BYTE(x)    do { if (Serial) { UART_DEBUG.print("0x"); if (x < 0x10) UART_DEBUG.print("0"); UART_DEBUG.print(x, HEX); } } while(0)
    #define DBG_BL0906_SEND_HEX32(x)   do { if (Serial) { UART_DEBUG.print("0x"); UART_DEBUG.print(x, HEX); } } while(0)
    #define DBG_Bl0906_SEND_DWORD(x)   do { if (Serial) UART_DEBUG.print(x); } while(0)
    #define DBG_Bl0906_SEND_FLOAT(x)   do { if (Serial) UART_DEBUG.print(x, 3); } while(0)
#else
	#define DBG_BL0906_SEND_STR(x)
    #define DBG_BL0906_SEND_STR_INFO(x)
    #define DBG_BL0906_SEND_STR_ERROR(x)
    #define DBG_BL0906_SEND_INT(x)
	#define DBG_BL0906_SEND_HEX(x)
	#define DBG_BL0906_SEND_BYTE(x)
    #define DBG_BL0906_SEND_HEX32(x)
    #define DBG_Bl0906_SEND_DWORD(x)
    #define DBG_Bl0906_SEND_FLOAT(x)
#endif

typeBl0906_handle_update_energy pvBl0906_handle_update_energy = NULL;
static UartService* p_uart_service = NULL;
static UartService* p_debug_uart = NULL; // UART để log debug
static uint8_t current_channel = 0;  // Kênh hiện tại (0-3)

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
const uint16_t timeout = 1000;  //Serial timeout[ms]
const float Vref = 1.097;  //[V]

const float Rf = 470*4;
const float Rv = 1.1;
const float Gain_v = 1;
const float Gain_i = 16;
const float Rl = 2;  // mOhm

static uint8_t cmd_is_running_register = REG_UNKNOWN;  // Register đang đọc
static measurement_value_t measurement_values[4];  // Mảng cho 4 kênh

bool manual_rx_flag = false;
uint8_t manual_rx_data[4] = { 0, 0, 0, 0 };

static current_correction_par_t  current_correction_par;
static gain_par_t gain_par[4] = {  // Mảng gain_par cho 4 kênh
	{ .value = 0, .set_gain_st_t_ms = 0 },
	{ .value = 0, .set_gain_st_t_ms = 0 },
	{ .value = 0, .set_gain_st_t_ms = 0 },
	{ .value = 0, .set_gain_st_t_ms = 0 }
};

// ========== SYNC (Blocking) Variables ==========
static bool sync_read_waiting = false;
static uint8_t sync_read_register = REG_UNKNOWN;
static uint8_t sync_read_response[BL0906_RX_LEN] = {0};
static bool sync_read_success = false;

/******************************************************************************/
/*                             PRIVATE FUNCS                                  */
/******************************************************************************/
static bool bl0906_write_register(uint8_t address, uint32_t data, UartService* uart_service);
static void bl0906_read_register(uint8_t address);
static void bl0906_set_gain_proc(void);  // Forward declaration

/******************************************************************************/
/*                        EXPORT FUNCTIONS DECLERATION                        */
/******************************************************************************/


/**
 * @func    bl0906_handdle_rx_manual
 * @brief
 * @param
 * @retval  None
 */
void bl0906_handdle_rx_manual(uint8_t* buff, uint8_t len)
{
	if(len >= sizeof(manual_rx_data)) {
		memcpy(manual_rx_data, buff, sizeof(manual_rx_data));
		manual_rx_flag = true;
	}
}

/**
 * @func    bl_0906_set_gain
 * @brief   Set gain register cho BL0906
 * @param   gain: Giá trị gain (ví dụ: 0x333300 hoặc 16)
 * @param   uart_service: UART service của kênh cần set (NULL = dùng UART hiện tại)
 * @retval  None
 */
void bl_0906_set_gain(uint32_t gain, UartService* uart_service)
{
	bl0906_write_register(GAIN_1, gain, uart_service);
}

/**
 * @func    bl0940_init
 * @brief
 * @param
 * @retval  None
 */
void bl0906_init(typeBl0906_handle_update_energy func, UartService* uart_service)
{
	p_uart_service = uart_service;
	
	if(func != NULL) {
		pvBl0906_handle_update_energy = func;
	}
	// For current correction when power on
	current_correction_par.start_time_ms = clock_time_ms();
	current_correction_par.is_running = true;
	current_correction_par.retry_start_time_ms = 0;
	foreach(i, NUMBER_RL) {
		current_correction_par.complete_flag[i] = false;
	}

	//bl0906_proc();
	//delay(50);  // Đợi ghi xong
	//bl0906_send_get_current();
}

/**
 * @func    bl0906_set_uart
 * @brief   Set UART tạm thời 
 để đọc từ kênh khác
 * @param   uart_service: UART service
 * @retval  None
 */
void bl0906_set_uart(UartService* uart_service)
{
	p_uart_service = uart_service;
	// Không set channel ở đây, sẽ set riêng
}

void bl0906_set_channel(uint8_t channel)
{
	if (channel < 4) {
		current_channel = channel;
		// Reset giá trị của kênh này trước khi đọc
		memset(&measurement_values[channel], 0, sizeof(measurement_value_t));
		measurement_values[channel].voltage_ok = false;
		measurement_values[channel].current_1_ok = false;
		measurement_values[channel].current_2_ok = false;
		measurement_values[channel].current_3_ok = false;
	}
}

/**
 * @func    bl0906_set_debug_uart
 * @brief   Set UART để log debug
 * @param   debug_uart: UART service để log debug
 * @retval  None
 */
void bl0906_set_debug_uart(UartService* debug_uart)
{
	p_debug_uart = debug_uart;
}

/**
 * @func    array_to_u24
 * @brief
 * @param
 * @retval  None
 */
static uint32_t array_to_u24(uint8_t* in)
{
	return (uint32_t)(in[0] | in[1] << 8 | in[2] << 16);
}

/**
 * @func    bl0940_update_energy
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_update_energy(m_type_enum type, float value)
{
	if(pvBl0906_handle_update_energy != NULL) {
		pvBl0906_handle_update_energy(type, value);
	}
}

/**
 * @func    bl0906_bias_correction
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_bias_correction(uint8_t addr, float measurements, float correction)
{
	uint8_t index;
	switch(addr) {
		case RMSOS_1:
			index = 0;
			break;
		case RMSOS_2:
			index = 1;
			break;

		case RMSOS_3:
			index = 2;
			break;

		default:
			return;
	}
	double  ki =  (12875 * Rl * Gain_i)/Vref;
	float i_rms0 = measurements * ki;
	float i_rms = correction * ki;
	int32_t value = (i_rms * i_rms - i_rms0 * i_rms0) / 256;

	uint8_t data[3];
	data[0] = value;
	data[1] = (value >> 8);
    if (value < 0) {
    	data[2] = (value >> 16) | 0b10000000;
    }
    else {
    	data[2] = (value >> 16);
    }
	bl0906_write_register(addr, value, NULL);

	current_correction_par.rmsos[index] = value&0x00FFFFFF;

	DBG_BL0906_SEND_STR("\n ^^^^^^^^^^^^^^^^^^^^^ RMSOS SET: ");
	DBG_BL0906_SEND_INT(index);
	DBG_BL0906_SEND_STR(", ");
	DBG_Bl0906_SEND_DWORD(value);
}

/**
 * @func    bl0906_is_correction_complete_or_timeout
 * @brief
 * @param
 * @retval  None
 */
bool bl0906_is_correction_complete_or_timeout(void)
{
	if(clock_time_exceed_ms(current_correction_par.start_time_ms, CURRENT_CORRECTION_TIMEOUT_MS)) {
		return true;
	}
	foreach(i, NUMBER_RL) {
		if(current_correction_par.complete_flag[i] == false) {
			return false;
		}
	}
	return true;
}


/**
 * @func    bl0906_handle_current_rsp
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_handle_current_rsp(uint8_t* par, uint8_t par_len)
{
	uint32_t data = array_to_u24(par);
	
	// Debug: Log raw data trước khi tính toán
	DBG_BL0906_SEND_STR_INFO("BL0906: Current raw data=");
	DBG_BL0906_SEND_HEX32(data);
	DBG_BL0906_SEND_STR_INFO(" (");
	DBG_Bl0906_SEND_DWORD(data);
	DBG_BL0906_SEND_STR_INFO(")");
	
	// Tính toán dòng điện
	// Công thức: I = data * Vref / (12875 * Rl * Gain_i)
	// Rl = 2 mOhm = 0.002 Ohm
	float ampere_value = (float)data * Vref / (12875.0f * Rl * Gain_i);
	float current_ma = ampere_value * 1000.0f;   // A to mA

	uint8_t index, correction_reg;
	switch(cmd_is_running_register) {
		case I1_RMS:
			index = 0;
			correction_reg = RMSOS_1;
			break;
		case I2_RMS:
			index = 1;
			correction_reg = RMSOS_2;
			break;
		case I3_RMS:
			index = 2;
			correction_reg = RMSOS_3;
			break;
		default:
			return;
	}
	
	// Debug: Log giá trị tính được
	DBG_BL0906_SEND_STR_INFO("\nBL0906: Current[");
	DBG_BL0906_SEND_INT(index);
	DBG_BL0906_SEND_STR_INFO("] raw data=");
	DBG_BL0906_SEND_HEX32(data);
	DBG_BL0906_SEND_STR_INFO(" (");
	DBG_Bl0906_SEND_DWORD(data);
	DBG_BL0906_SEND_STR_INFO(")");
	DBG_BL0906_SEND_STR_INFO("\nBL0906: Current[");
	DBG_BL0906_SEND_INT(index);
	DBG_BL0906_SEND_STR_INFO("] calc: data=");
	DBG_Bl0906_SEND_DWORD(data);
	DBG_BL0906_SEND_STR_INFO("\nBL0906: Vref=");
	DBG_Bl0906_SEND_FLOAT(Vref);
	DBG_BL0906_SEND_STR_INFO(", Rl=");
	DBG_Bl0906_SEND_FLOAT(Rl);
	DBG_BL0906_SEND_STR_INFO(", Gain_i=");
	DBG_BL0906_SEND_INT(Gain_i);
	DBG_BL0906_SEND_STR_INFO("\nBL0906: Current[");
	DBG_BL0906_SEND_INT(index);
	DBG_BL0906_SEND_STR_INFO("] result = ");
	DBG_Bl0906_SEND_FLOAT(current_ma);
	DBG_BL0906_SEND_STR_INFO(" mA");
	
	// Lưu giá trị vào mảng theo index và channel hiện tại
	measurement_values[current_channel].current[index] = current_ma;
	
	if(current_correction_par.is_running == true) {
		if(current_correction_par.complete_flag[index] == false) {
			if(measurement_values[current_channel].current[index] == 0) {
				current_correction_par.complete_flag[index] = true;
			}
			else {
				if(measurement_values[current_channel].current[index] <= CURRENT_OFFSET_MA_MAX) {
					bl0906_bias_correction(correction_reg, ampere_value, 0);
					bl0906_read_register(correction_reg);
				}
			}
		}
	}
	bl0906_update_energy(TYPE_CURRENT, measurement_values[current_channel].current[index]);

	DBG_BL0906_SEND_STR("\n*************************");
	DBG_BL0906_SEND_STR("\n __CURRENT__:");
	DBG_BL0906_SEND_BYTE(index);
	DBG_BL0906_SEND_STR(", ");
	DBG_BL0906_SEND_INT((uint16_t)measurement_values[current_channel].current[index]);
}

/**
 * @func    bl0906_handle_rmsos_rsp
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_handle_rmsos_rsp(uint8_t* par, uint8_t par_len)
{
	uint32_t rmsos = array_to_u24(par);
	uint8_t index;
	switch(cmd_is_running_register) {
		case RMSOS_1:
			index = 0;
			break;
		case RMSOS_2:
			index = 1;
			break;

		case RMSOS_3:
			index = 2;
			break;

		default:
			return;
	}

	DBG_BL0906_SEND_STR("\n RESPONSE RMSOS: ");
	DBG_BL0906_SEND_INT(index);
	DBG_BL0906_SEND_STR(", ");
	DBG_Bl0906_SEND_DWORD(rmsos);

	if(current_correction_par.rmsos[index] == rmsos) {
		current_correction_par.complete_flag[index] = true;
		DBG_BL0906_SEND_STR("\n RMSOS set complete: ");
		DBG_BL0906_SEND_INT(index);
		DBG_BL0906_SEND_STR(", ");
		DBG_Bl0906_SEND_DWORD(current_correction_par.rmsos[index]);
	}
}

/**
 * @func    bl0906_handle_voltage_rsp
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_handle_voltage_rsp(uint8_t* par, uint8_t par_len)
{
	uint32_t data = array_to_u24(par);
	
	// Debug: Log raw data trước khi tính toán
	DBG_BL0906_SEND_STR_INFO("BL0906: Voltage raw data=");
	DBG_BL0906_SEND_HEX32(data);
	DBG_BL0906_SEND_STR_INFO(" (");
	DBG_Bl0906_SEND_DWORD(data);
	DBG_BL0906_SEND_STR_INFO(")");
	
	// Tính toán điện áp
	// Công thức: V = data * Vref * (Rf + Rv) / (13162 * Rv * Gain_v * 1000)
	float voltage_calc = (float)data * Vref * (Rf + Rv) / (13162.0f * Rv * Gain_v * 1000.0f);
	measurement_values[current_channel].voltage = voltage_calc;
	
	// Debug: Log các giá trị trung gian
	DBG_BL0906_SEND_STR_INFO("\nBL0906: Voltage raw data=");
	DBG_BL0906_SEND_HEX32(data);
	DBG_BL0906_SEND_STR_INFO(" (");
	DBG_Bl0906_SEND_DWORD(data);
	DBG_BL0906_SEND_STR_INFO(")");
	DBG_BL0906_SEND_STR_INFO("\nBL0906: Voltage calc: data=");
	DBG_Bl0906_SEND_DWORD(data);
	DBG_BL0906_SEND_STR_INFO("\nBL0906: Vref=");
	DBG_Bl0906_SEND_FLOAT(Vref);
	DBG_BL0906_SEND_STR_INFO(", Rf=");
	DBG_Bl0906_SEND_FLOAT(Rf);
	DBG_BL0906_SEND_STR_INFO(", Rv=");
	DBG_Bl0906_SEND_FLOAT(Rv);
	DBG_BL0906_SEND_STR_INFO(", Gain_v=");
	DBG_Bl0906_SEND_FLOAT(Gain_v);
	DBG_BL0906_SEND_STR_INFO("\nBL0906: Voltage result = ");
	DBG_Bl0906_SEND_FLOAT(voltage_calc);
	DBG_BL0906_SEND_STR_INFO(" V");
	
	bl0906_update_energy(TYPE_VOLTAGE, measurement_values[current_channel].voltage);
	DBG_BL0906_SEND_STR("\n __VOLTAGE__:");
	DBG_BL0906_SEND_INT((uint16_t)measurement_values[current_channel].voltage);
	DBG_BL0906_SEND_STR(", ");
	DBG_Bl0906_SEND_FLOAT(measurement_values[current_channel].voltage);
}

/**
 * @func    bl0906_handle_active_power_rsp
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_handle_active_power_rsp(uint8_t* par, uint8_t par_len)
{
	uint32_t data = array_to_u24(par);
	bool sign_bit = (data >> 23) & 0x01;
	float power_value = 0;
	if (sign_bit == 1) {
		DBG_BL0906_SEND_STR("\n Power Unused");
		return;
	}
	else {
		power_value = (float)data * Vref * Vref * (Rf + Rv) /
			(40.4125 * Rl*Rv*Gain_i * Gain_v*1000);
	}
	
	
	// Xác định index dựa vào register
	uint8_t index = 0;
	switch(cmd_is_running_register) {
		case WATT_1:
			index = 0;
			break;
		case WATT_2:
			index = 1;
			break;
		case WATT_3:
			index = 2;
			break;
		default:
			return;
	}
	
	// Lưu giá trị vào mảng theo index
	measurement_values[current_channel].active_power[index] = power_value;
	bl0906_update_energy(TYPE_ACTIVE_POWER, measurement_values[current_channel].active_power[index]);
	DBG_BL0906_SEND_STR("\n __ACTIVE_POWER__:");
	DBG_BL0906_SEND_BYTE(index);
	DBG_BL0906_SEND_STR(", ");
	DBG_BL0906_SEND_INT((uint16_t)measurement_values[current_channel].active_power[index]);
	DBG_BL0906_SEND_STR(", ");
	DBG_Bl0906_SEND_FLOAT(measurement_values[current_channel].active_power[index]);
}

/**
 * @func    bl0906_handle_temperature_rsp
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_handle_temperature_rsp(uint8_t* par, uint8_t par_len)
{
	int32_t data = (int32_t)array_to_u24(par) & 0x03FF;
	measurement_values[current_channel].temperature =  \
			(data - 64)*12.5/59-40;
	bl0906_update_energy(TYPE_TEMPERATURE, measurement_values[current_channel].temperature);
}

/**
 * @func    bl0906_handle_temperature_rsp
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_handle_gain_rsp(uint8_t* par, uint8_t par_len)
{
	// Cập nhật gain_par cho kênh hiện tại
	gain_par[current_channel].value = array_to_u24(par);
	DBG_BL0906_SEND_STR_INFO("\nBL0906: Channel ");
	DBG_BL0906_SEND_INT(current_channel);
	DBG_BL0906_SEND_STR_INFO(" Gain raw data=");
	DBG_BL0906_SEND_HEX32(gain_par[current_channel].value);
	DBG_BL0906_SEND_STR_INFO(" (");
	DBG_Bl0906_SEND_DWORD(gain_par[current_channel].value);
	DBG_BL0906_SEND_STR_INFO(")");
	DBG_BL0906_SEND_STR_INFO("\nBL0906: Gain result Duong= ");
	DBG_Bl0906_SEND_DWORD(gain_par[current_channel].value);
	DBG_BL0906_SEND_STR_INFO(")");
}


/**
 * @func    bl0906_init
 * @brief
 * @param
 * @retval  None
 */
void bl0906_measurenment_start(uint16_t m_mask)
{
	foreach(i, BL0904_MEASUREMENT_MAX)
	{
		uint16_t tmp = (m_mask & (uint16_t)(1 << i));
		if(!tmp) continue;
		if(tmp == BIT_MASK_CURRENT) {
			bl0906_send_get_current();
		}
		else if(tmp == BIT_MASK_VOLTAGE) {
			bl0906_get_voltage();
		}
		else if(tmp == BIT_MASK_ACTIVE_POWER) {
			bl0906_get_active_power();
		}
		else if(tmp == BIT_MASK_ACTIVE_ENERGY) {
			bl0906_get_active_energy();
		}
		else if(tmp == BIT_MASK_POWER_FACTOR) {
			bl0906_get_power_factor();
		}
		else if(tmp == BIT_MASK_TEMPERATURE) {
			bl0906_get_temperature();
		}
	}
}

/**
 * @func    bl0906_current_correction_proc
 * @brief   Note, make sure that all relays are off, otherwise this process may cause errors.
 * @param
 * @retval  None
 */
static void bl0906_current_correction_proc(void)
{
	bool complete = true;
	if(current_correction_par.is_running == false) {
		return;
	}
	if(clock_time_exceed_ms(current_correction_par.start_time_ms, CURRENT_CORRECTION_TIMEOUT_MS)) {
		current_correction_par.is_running = false;
		return;
	}
	foreach(i, NUMBER_RL) {
		if(current_correction_par.complete_flag[i] == false) {
			if(clock_time_exceed_ms(  \
					current_correction_par.retry_start_time_ms, CURRENT_CORRECTION_RETRY_INTERVAl_MS)  \
						|| (current_correction_par.retry_start_time_ms == 0)) {
				current_correction_par.retry_start_time_ms = clock_time_ms();
				if(current_correction_par.retry_start_time_ms == 0) {
					current_correction_par.retry_start_time_ms = 1;
				}
				bl0906_send_get_current();
				return;
			}
			complete = false;
		}
	}
	if(complete == true) {
		current_correction_par.is_running = false;
	}
}

/**
 * @func    bl0906_set_gain_proc
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_set_gain_proc(void)
{
	// Kiểm tra channel hợp lệ
	if(current_channel >= 4) {
		return;
	}
	
	DBG_BL0906_SEND_STR_INFO("\nbl0906_set_gain_proc - Channel ");
	DBG_BL0906_SEND_INT(current_channel);
	DBG_BL0906_SEND_STR_INFO("\ngain_par[");
	DBG_BL0906_SEND_INT(current_channel);
	DBG_BL0906_SEND_STR_INFO("].value=");
	DBG_Bl0906_SEND_DWORD(gain_par[current_channel].value);
	DBG_BL0906_SEND_STR_INFO("\nGAIN_1_DEFAULT_VALUE=");
	DBG_Bl0906_SEND_DWORD(GAIN_1_DEFAULT_VALUE);
	
	// Nếu chưa đọc gain lần nào cho kênh này, đọc trước để biết giá trị hiện tại
	if(gain_par[current_channel].value == 0 && gain_par[current_channel].set_gain_st_t_ms == 0) {
		DBG_BL0906_SEND_STR_INFO("\nBL0906: Channel ");
		DBG_BL0906_SEND_INT(current_channel);
		DBG_BL0906_SEND_STR_INFO(" First time - reading GAIN_1");
		bl0906_read_register(GAIN_1);
	}
	
	if(gain_par[current_channel].value != GAIN_1_DEFAULT_VALUE) {
		DBG_BL0906_SEND_STR_INFO("\nBL0906: Channel ");
		DBG_BL0906_SEND_INT(current_channel);
		DBG_BL0906_SEND_STR_INFO(" Gain not equal to default value");
		// Cho phép set gain ngay lần đầu (set_gain_st_t_ms = 0) hoặc sau interval
		if(gain_par[current_channel].set_gain_st_t_ms == 0 || clock_time_exceed_ms(gain_par[current_channel].set_gain_st_t_ms, SET_GAIN_INTERVAL_MS)) {
			// Kiểm tra UART trước khi set gain
			if (p_uart_service == NULL || p_uart_service->getSerial() == NULL) {
				DBG_BL0906_SEND_STR_ERROR("\nBL0906: Channel ");
				DBG_BL0906_SEND_INT(current_channel);
				DBG_BL0906_SEND_STR_ERROR(" UART is NULL! Cannot set gain.");
				return;
			}
			
			// Retry với verify để đảm bảo 100% thành công
			const uint8_t MAX_RETRY = 3;
			bool set_success = false;
			
			for(uint8_t retry = 0; retry < MAX_RETRY && !set_success; retry++) {
				DBG_BL0906_SEND_STR_INFO("\nBL0906: Channel ");
				DBG_BL0906_SEND_INT(current_channel);
				DBG_BL0906_SEND_STR_INFO(" Set gain attempt ");
				DBG_BL0906_SEND_INT(retry + 1);
				UART_DEBUG.print("DEBUG_UART: ");
				UART_DEBUG.println(retry+1);
				
				// Flush UART trước khi set gain để đảm bảo buffer sạch
				p_uart_service->getSerial()->flush();
				
				// Set gain
				bl_0906_set_gain(GAIN_1_DEFAULT_VALUE, NULL);
				delay(5);  // Đợi ghi xong
				
				// Verify: Đọc lại để kiểm tra
				bl0906_read_register(GAIN_1);
				
				// Kiểm tra xem gain đã được set đúng chưa
				if(gain_par[current_channel].value == GAIN_1_DEFAULT_VALUE) {
					set_success = true;
					DBG_BL0906_SEND_STR_INFO("\nBL0906: Channel ");
					DBG_BL0906_SEND_INT(current_channel);
					DBG_BL0906_SEND_STR_INFO(" Set gain SUCCESS");
				} else {
					DBG_BL0906_SEND_STR_INFO("\nBL0906: Channel ");
					DBG_BL0906_SEND_INT(current_channel);
					DBG_BL0906_SEND_STR_INFO(" Set gain FAILED (read=");
					DBG_Bl0906_SEND_DWORD(gain_par[current_channel].value);
					DBG_BL0906_SEND_STR_INFO("), retry...");
					//delay(50);  // Đợi trước khi retry
				}
			}
			
			if(!set_success) {
				DBG_BL0906_SEND_STR_ERROR("\nBL0906: Channel ");
				DBG_BL0906_SEND_INT(current_channel);
				DBG_BL0906_SEND_STR_ERROR(" Set gain FAILED after ");
				DBG_BL0906_SEND_INT(MAX_RETRY);
				DBG_BL0906_SEND_STR_ERROR(" attempts!");
			}
			
			gain_par[current_channel].set_gain_st_t_ms = clock_time_ms();
		}
	} else {
		DBG_BL0906_SEND_STR_INFO("\nBL0906: Channel ");
		DBG_BL0906_SEND_INT(current_channel);
		DBG_BL0906_SEND_STR_INFO(" Gain already correct");
	}
}
void bl0906_proc(void)
{
	bl0906_set_gain_proc();

}
/**
 * @func    _culcCheckSum
 * @brief
 * @param
 * @retval  None
 */
uint8_t _culcCheckSum(uint8_t *tx_data, int tx_len, uint8_t *rx_data, int rx_len)
{
	uint8_t checksum = 0;
	for(uint8_t i = 0; i < tx_len; i++) {
		checksum += tx_data[i];
	}
	for(uint8_t i = 0; i < rx_len; i++) {
		checksum += rx_data[i];
	}
	checksum = checksum^0xFF;
	return checksum;
}


/**
 * @func    bl0940_write_register
 * @brief
 * @param
 * @retval  None
 */
static bool bl0906_write_register(uint8_t address, uint32_t data, UartService* uart_service)
{
	// Nếu không chỉ định UART, dùng UART hiện tại
	UartService* target_uart = (uart_service != NULL) ? uart_service : p_uart_service;
	
	if (target_uart == NULL || target_uart->getSerial() == NULL) {
		return false;
	}
	
	HardwareSerial* serial = target_uart->getSerial();
	
	// Remove write protection
	uint8_t tx_wrprot[] = { BL0906_WRITE_COMMAND, 0x9e, 0x55, 0x55, 0x00, 0xb7};
	serial->write(tx_wrprot, sizeof(tx_wrprot));
	serial->flush();
	delay(10);
	
	// Write Register
    uint8_t tx_data[6] = {BL0906_WRITE_COMMAND, address, (uint8_t)(data), (uint8_t)(data >> 8), (uint8_t)(data >> 16)};
    tx_data[5] = _culcCheckSum(&tx_data[1], sizeof(tx_data) - 2, 0, 0);
    serial->write(tx_data, sizeof(tx_data));
    serial->flush();
    delay(10);

    // Enable write protection
	uint8_t tx_read_only[] = { BL0906_WRITE_COMMAND, 0x9e, 0x00, 0x00, 0x00, 0x61};
	serial->write(tx_read_only, sizeof(tx_read_only));
	serial->flush();
    return true;
}

/**
 * @func    bl0906_read_register
 * @brief   Đọc register thủ công: gửi lệnh qua UART, đọc phản hồi và xử lý ngay
 * @param   address: Register address
 * @retval  None
 */
static void bl0906_read_register(uint8_t address)
{
	if (p_uart_service == NULL || p_uart_service->getSerial() == NULL) {
		return;
	}
	
	HardwareSerial* serial = p_uart_service->getSerial();
	
	// Xóa buffer trước khi đọc để tránh dữ liệu cũ
	while(serial->available() > 0) {
		serial->read();
	}
	
	// Gửi lệnh đọc register
	uint8_t tx_data[] = { BL0906_READ_COMMAND, address };
	serial->write(tx_data, sizeof(tx_data));
	serial->flush(); // Đảm bảo dữ liệu đã được gửi
	
	// Log raw data gửi đi (debug)
	DBG_BL0906_SEND_STR_INFO("\nBL0906: TX [");
	DBG_BL0906_SEND_BYTE(tx_data[0]);
	DBG_BL0906_SEND_STR_INFO(" ");
	DBG_BL0906_SEND_BYTE(tx_data[1]);
	DBG_BL0906_SEND_STR_INFO("] reg=");
	DBG_BL0906_SEND_BYTE(address);
	
	// Đợi một chút để thiết bị có thời gian phản hồi
	// Khi debug bật, các print statements trên tạo delay tự nhiên (~10-20ms)
	// Khi debug tắt, cần delay này để đảm bảo BL0906 có đủ thời gian xử lý
	delay(15);  // Tăng từ 5ms lên 15ms để bù thời gian debug output 
	
	// Đợi cho đến khi có đủ bytes trong buffer hoặc timeout
	uint32_t start_time = millis();
	uint32_t timeout_ms = 50; // Timeout 500ms (tăng lên để đợi lâu hơn)
	uint8_t available_bytes = 0;
	while ((available_bytes = serial->available()) < BL0906_RX_LEN) {
		if ((millis() - start_time) >= timeout_ms) {
			// Timeout - log để debug
			DBG_BL0906_SEND_STR_ERROR("\nBL0906: Timeout reading reg ");
			DBG_BL0906_SEND_BYTE(address);
			DBG_BL0906_SEND_STR_ERROR(" (got ");
			DBG_BL0906_SEND_INT(available_bytes);
			DBG_BL0906_SEND_STR_ERROR(" bytes, waited ");
			DBG_Bl0906_SEND_DWORD(millis() - start_time);
			DBG_BL0906_SEND_STR_ERROR(" ms)");
			// Log bất kỳ dữ liệu nào đã nhận được
			if (available_bytes > 0) {
				uint8_t partial[10];
				uint8_t partial_read = serial->readBytes(partial, available_bytes);
				DBG_BL0906_SEND_STR_ERROR("\nBL0906: Partial RX [");
				DBG_BL0906_SEND_INT(partial_read);
				DBG_BL0906_SEND_STR_ERROR(" bytes]: ");
				for (uint8_t i = 0; i < partial_read; i++) {
					DBG_BL0906_SEND_BYTE(partial[i]);
					DBG_BL0906_SEND_STR_ERROR(" ");
				}
			}
			return;
		}
		delay(5); // Đợi 5ms mỗi lần (giảm số lần check)
	}
	
	// Đọc phản hồi (4 bytes: 3 bytes data + 1 byte checksum)
	uint8_t rx_data[BL0906_RX_LEN] = {0};
	uint8_t bytes_read = serial->readBytes(rx_data, BL0906_RX_LEN);
	
	// Log raw data nhận được (debug)
	DBG_BL0906_SEND_STR_INFO("\nBL0906: RX [");
	DBG_BL0906_SEND_INT(bytes_read);
	DBG_BL0906_SEND_STR_INFO(" bytes]: ");
	DBG_BL0906_SEND_BYTE(rx_data[0]);
	DBG_BL0906_SEND_STR_INFO(" ");
	DBG_BL0906_SEND_BYTE(rx_data[1]);
	DBG_BL0906_SEND_STR_INFO(" ");
	DBG_BL0906_SEND_BYTE(rx_data[2]);
	DBG_BL0906_SEND_STR_INFO(" ");
	DBG_BL0906_SEND_BYTE(rx_data[3]);
	
	if (bytes_read != BL0906_RX_LEN) {
		// Không đủ dữ liệu (không nên xảy ra vì đã kiểm tra available)
		DBG_BL0906_SEND_STR_ERROR("\nBL0906: Incomplete data - expected ");
		DBG_BL0906_SEND_INT(BL0906_RX_LEN);
		DBG_BL0906_SEND_STR_ERROR(", got ");
		DBG_BL0906_SEND_INT(bytes_read);
		return;
	}
	
	// Kiểm tra checksum
	uint8_t checksum = address;
	for(uint8_t i = 0; i < BL0906_RX_LEN - 1; i++) {
		checksum += rx_data[i];
	}
	checksum = checksum ^ 0xFF;
	
	if(checksum != rx_data[BL0906_RX_LEN - 1]) {
		// Checksum sai - log để debug
		DBG_BL0906_SEND_STR_ERROR("\nBL0906: Checksum error reg ");
		DBG_BL0906_SEND_BYTE(address);
		DBG_BL0906_SEND_STR_ERROR(" (calc=");
		DBG_BL0906_SEND_BYTE(checksum);
		DBG_BL0906_SEND_STR_ERROR(" recv=");
		DBG_BL0906_SEND_BYTE(rx_data[BL0906_RX_LEN - 1]);
		DBG_BL0906_SEND_STR_ERROR(")");
		return;
	}
	
	// Xác định handler tương ứng và gọi xử lý
	p_func_handle handler = NULL;
	switch(address) {
		case I1_RMS:
			handler = bl0906_handle_current_rsp;
			break;
		case I2_RMS:
			handler = bl0906_handle_current_rsp;
			break;
		case I3_RMS:
			handler = bl0906_handle_current_rsp;
			break;
			
		case RMSOS_1:
		case RMSOS_2:
		case RMSOS_3:
			handler = bl0906_handle_rmsos_rsp;
			break;
			
		case V_RMS:
			handler = bl0906_handle_voltage_rsp;
			break;
			
		case WATT_1:
		handler = bl0906_handle_active_power_rsp;
		break;
		
		case WATT_2:
		handler = bl0906_handle_active_power_rsp;
		break;
		
		case WATT_3:
			handler = bl0906_handle_active_power_rsp;
			break;
			
		case TPS:
			handler = bl0906_handle_temperature_rsp;
			break;
			
		case GAIN_1:
			handler = bl0906_handle_gain_rsp;
			break;
			
		default:
			// Không có handler cho register này
			return;
	}
	
	// Gọi handler để xử lý dữ liệu (3 bytes data, không bao gồm checksum)
	if(handler != NULL) {
		// Cần set cmd_is_running_register để handler biết đang đọc register nào
		cmd_is_running_register = address;
		handler(rx_data, BL0906_RX_LEN - 1);
		cmd_is_running_register = REG_UNKNOWN;
		
		// Log sau khi xử lý để debug
		uint32_t data_value = array_to_u24(rx_data);
		DBG_BL0906_SEND_STR_INFO("\nBL0906: Processed reg ");
		DBG_BL0906_SEND_BYTE(address);
		DBG_BL0906_SEND_STR_INFO(", data=");
		DBG_BL0906_SEND_HEX32(data_value);
		DBG_BL0906_SEND_STR_INFO(" (");
		DBG_Bl0906_SEND_DWORD(data_value);
		DBG_BL0906_SEND_STR_INFO(")");
		
		// Log giá trị đã lưu tùy theo register
		if (address == I1_RMS || address == I2_RMS || address == I3_RMS) {
			uint8_t idx = (address == I1_RMS) ? 0 : (address == I2_RMS) ? 1 : 2;
			DBG_BL0906_SEND_STR_INFO("\nBL0906: Current[");
			DBG_BL0906_SEND_INT(idx);
			DBG_BL0906_SEND_STR_INFO("] = ");
			DBG_Bl0906_SEND_FLOAT(measurement_values[current_channel].current[idx]);
			DBG_BL0906_SEND_STR_INFO(" mA");
		} else if (address == V_RMS) {
			DBG_BL0906_SEND_STR_INFO("\nBL0906: Voltage = ");
			DBG_Bl0906_SEND_FLOAT(measurement_values[current_channel].voltage);
			DBG_BL0906_SEND_STR_INFO(" V");
		}
	} else {
		// Không có handler cho register này
		DBG_BL0906_SEND_STR_ERROR("\nBL0906: No handler for reg ");
		DBG_BL0906_SEND_BYTE(address);
	}
}

/**
 * @func    bl0940_get_current
 * @brief
 * @param
 * @retval  None
 */
void bl0906_send_get_current(void)
{
	bl0906_read_register(I1_RMS);
	delay(10);  // Delay giữa các lần đọc để BL0906 có thời gian xử lý
	bl0906_read_register(I2_RMS);
	delay(10);  // Delay giữa các lần đọc để BL0906 có thời gian xử lý
	bl0906_read_register(I3_RMS);
}

/**
 * @func    bl0906_get_voltage
 * @brief
 * @param
 * @retval  None
 */
void bl0906_get_voltage(void)
{
	bl0906_read_register(V_RMS);
}

/**
 * @func    bl0906_get_active_power
 * @brief
 * @param
 * @retval  None
 */
void bl0906_get_active_power(void)
{
	bl0906_read_register(WATT_1);
	delay(10);  // Delay giữa các lần đọc để BL0906 có thời gian xử lý
	bl0906_read_register(WATT_2);
	delay(10);  // Delay giữa các lần đọc để BL0906 có thời gian xử lý
	bl0906_read_register(WATT_3);
}

/**
 * @func    bl0906_get_active_energy
 * @brief
 * @param
 * @retval  None
 */
void bl0906_get_active_energy(void)
{
	// bl0906_read_register(ACTIVE_ENERGY_REG);
}

/**
 * @func    bl0906_get_power_factor
 * @brief
 * @param
 * @retval  None
 */
void bl0906_get_power_factor(void)
{
	//bl0906_read_register(POWER_FACTOR_REG);
}

/**
 * @func    bl0906_get_temperature
 * @brief
 * @param
 * @retval  None
 */
void bl0906_get_temperature(void)
{
	bl0906_read_register(TPS);
}

/**
 * @func    bl0906_reset
 * @brief
 * @param
 * @retval  None
 */
bool bl0906_reset(void)
{
	if (false == bl0906_write_register(SOFT_RESET, 0x5A5A5A, NULL)) {
		DBG_BL0906_SEND_STR("Can not write SOFT_RESET register.");
		return false;
	}
	delay(500);
	return true;
}

/******************************************************************************/
/* ========== Getter Functions - Lấy giá trị đo được ==========              */
/******************************************************************************/

/**
 * @func    bl0906_get_current_value
 * @brief   Lấy giá trị dòng điện hiện tại (mA)
 * @param   channel: Kênh dòng điện (0=I1, 1=I2, 2=I3)
 * @retval  Giá trị dòng điện (mA)
 */
float bl0906_get_current_value(uint8_t channel)
{
	if (channel >= 3) {
		return 0.0f;
	}
	return measurement_values[current_channel].current[channel];
}

/**
 * @func    bl0906_get_voltage_value
 * @brief   Lấy giá trị điện áp hiện tại (V)
 * @param   None
 * @retval  Giá trị điện áp (V)
 */
float bl0906_get_voltage_value(void)
{
	return measurement_values[current_channel].voltage;
}

/**
 * @func    bl0906_get_active_power_value
 * @brief   Lấy giá trị công suất tác dụng hiện tại (W)
 * @param   channel: Kênh công suất (0=WATT_1, 1=WATT_2, 2=WATT_3)
 * @retval  Giá trị công suất (W)
 */
float bl0906_get_active_power_value(uint8_t channel)
{
	if (channel >= 3) {
		return 0.0f;
	}
	return measurement_values[current_channel].active_power[channel];
}

/**
 * @func    bl0906_get_active_energy_value
 * @brief   Lấy giá trị năng lượng tác dụng hiện tại
 * @param   None
 * @retval  Giá trị năng lượng
 */
float bl0906_get_active_energy_value(void)
{
	return measurement_values[current_channel].active_energy;
}

/**
 * @func    bl0906_get_power_factor_value
 * @brief   Lấy giá trị hệ số công suất hiện tại
 * @param   None
 * @retval  Giá trị hệ số công suất
 */
float bl0906_get_power_factor_value(void)
{
	return measurement_values[current_channel].power_factor;
}

/**
 * @func    bl0906_get_temperature_value
 * @brief   Lấy giá trị nhiệt độ hiện tại (°C)
 * @param   None
 * @retval  Giá trị nhiệt độ (°C)
 */
float bl0906_get_temperature_value(void)
{
	return measurement_values[current_channel].temperature;
}

/**
 * @func    bl0906_reset_measurements
 * @brief   Reset tất cả giá trị đo về 0 (dùng trước khi đọc kênh mới)
 * @param   None
 * @retval  None
 */
void bl0906_reset_measurements(void)
{
	// Reset giá trị của kênh hiện tại
	memset(&measurement_values[current_channel], 0, sizeof(measurement_value_t));
	// Reset các flag OK về false
	measurement_values[current_channel].voltage_ok = false;
	measurement_values[current_channel].current_1_ok = false;
	measurement_values[current_channel].current_2_ok = false;
	measurement_values[current_channel].current_3_ok = false;
	measurement_values[current_channel].active_power[0] = 0;
	measurement_values[current_channel].active_power[1] = 0;
	measurement_values[current_channel].active_power[2] = 0;
	
}

/**
 * @func    bl0906_get_all_measurements
 * @brief   Lấy tất cả các giá trị đo được
 * @param   None
 * @retval  Struct chứa tất cả giá trị đo được
 */
measurement_value_t bl0906_get_all_measurements(void)
{
	return measurement_values[current_channel];
}

/******************************************************************************/
/* ========== SYNC (Blocking) Functions ==========                            */
/******************************************************************************/

/**
 * @func    bl0906_read_register_sync
 * @brief   Đọc register đồng bộ: gửi cmd → đợi response → return
 * @param   address: Register address
 * @param   rx_data: Buffer để lưu 3 bytes data (không bao gồm checksum)
 * @param   timeout_ms: Timeout tính bằng milliseconds
 * @retval  true nếu thành công, false nếu timeout hoặc lỗi
 */
bool bl0906_read_register_sync(uint8_t address, uint8_t* rx_data, uint32_t timeout_ms)
{
	if (rx_data == NULL) {
		return false;
	}
	
	// Khởi tạo
	sync_read_waiting = true;
	sync_read_register = address;
	sync_read_success = false;
	memset(sync_read_response, 0, sizeof(sync_read_response));
	
	// Gửi command
	uint8_t tx_data[] = { BL0906_READ_COMMAND, address };
		if (p_uart_service != NULL && p_uart_service->getSerial() != NULL) {
			p_uart_service->getSerial()->write(tx_data, sizeof(tx_data));
			p_uart_service->getSerial()->flush();
		}
	
	// Đợi response
	uint32_t start_time = clock_time_ms();
	while (sync_read_waiting) {
		if (clock_time_exceed_ms(start_time, timeout_ms)) {
			sync_read_waiting = false;
			return false; // Timeout
		}
		// Cần gọi bl0906_handle_serial_rx_message() ở đâu đó để nhận data
		// delay nhỏ để không block quá nhiều
		delay(1);
	}
	
	if (sync_read_success) {
		memcpy(rx_data, sync_read_response, 3); // Copy 3 bytes data
		return true;
	}
	
	return false;
}

/**
 * @func    bl0906_read_register_sync_u24
 * @brief   Đọc register đồng bộ và trả về giá trị u24
 * @param   address: Register address
 * @param   timeout_ms: Timeout tính bằng milliseconds
 * @retval  Giá trị u24, 0 nếu lỗi
 */
uint32_t bl0906_read_register_sync_u24(uint8_t address, uint32_t timeout_ms)
{
	uint8_t rx_data[3] = {0};
	if (bl0906_read_register_sync(address, rx_data, timeout_ms)) {
		return array_to_u24(rx_data);
	}
	return 0;
}

