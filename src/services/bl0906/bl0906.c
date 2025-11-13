/*
 * bl0940.c
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
#include "fifo.h"
#include "../uart/uart_service.h"
#ifdef  BL0906_DBG_EN
	#define DBG_BL0906_SEND_STR(x)     Dbg_sendString((int8_t*)x)
	#define DBG_BL0906_SEND_INT(x)     Dbg_sendInt(x)
	#define DBG_BL0906_SEND_HEX(x)     Dbg_sendHex(x)
	#define DBG_BL0906_SEND_BYTE(x)    Dbg_sendOneByteHex(x)
    #define DBG_BL0906_SEND_HEX32(x)   Dbg_sendHex32(x)
    #define DBG_Bl0906_SEND_DWORD(x)   Dbg_sendDword(x)
    #define DBG_Bl0906_SEND_FLOAT(x)   //Dbg_sendFloat(x)
#else
	#define DBG_BL0906_SEND_STR(x)
	#define DBG_BL0906_SEND_INT(x)
	#define DBG_BL0906_SEND_HEX(x)
	#define DBG_BL0906_SEND_BYTE(x)
    #define DBG_BL0906_SEND_HEX32(x)
    #define DBG_Bl0906_SEND_DWORD(x)
    #define DBG_Bl0906_SEND_FLOAT(x)
#endif

typeBl0906_handle_update_energy pvBl0906_handle_update_energy = NULL;
static UartService* p_uart_service = NULL;

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
const uint16_t timeout = 1000;  //Serial timeout[ms]
const float Vref = 1.097;  //[V]

const float Rf = 470*4;
const float Rv = 1;
const float Gain_v = 1;
const float Gain_i = 16;
const float Rl = 2;  // mOhm

static bl0906_read_cmd_t cmd_is_running = { .id_register = REG_UNKNOWN };
static Fifo_t fifo_bl0906_cmd;
static bl0906_read_cmd_t buffer_bl0906_cmds[BL0906_BUF_CMD_SIZE];
static measurement_value_t measurement_value;

bool manual_rx_flag = false;
uint8_t manual_rx_data[4] = { 0, 0, 0, 0 };

static current_correction_par_t  current_correction_par;
static gain_par_t gain_par = { .value = 0, .set_gain_st_t_ms = 0 };

// ========== SYNC (Blocking) Variables ==========
static bool sync_read_waiting = false;
static uint8_t sync_read_register = REG_UNKNOWN;
static uint8_t sync_read_response[BL0906_RX_LEN] = {0};
static bool sync_read_success = false;

/******************************************************************************/
/*                             PRIVATE FUNCS                                  */
/******************************************************************************/
static void bl0906_push_msg(uint8_t *par, uint8_t par_len);
static bool bl0906_write_register(uint8_t address, uint32_t data);
static void bl0906_read_register(uint8_t address);

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
 * @brief
 * @param
 * @retval  None
 */
void bl_0906_set_gain(uint32_t gain)
{
	bl0906_write_register(GAIN_1, gain);
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
	
	FifoInit(&fifo_bl0906_cmd,  \
		&buffer_bl0906_cmds, sizeof(bl0906_read_cmd_t), BL0906_BUF_CMD_SIZE);
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
	bl0906_send_get_current();
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
	bl0906_write_register(addr, value);

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
	measurement_value.current = (float)data * Vref / (12875 * Rl * Gain_i);
	float ampere_value = measurement_value.current;
	measurement_value.current *= 1000;   // A to mA

	uint8_t index, correction_reg;
	switch(cmd_is_running.id_register) {
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
	if(current_correction_par.is_running == true) {
		if(current_correction_par.complete_flag[index] == false) {
			if(measurement_value.current == 0) {
				current_correction_par.complete_flag[index] = true;
			}
			else {
				if(measurement_value.current <= CURRENT_OFFSET_MA_MAX) {
					bl0906_bias_correction(correction_reg, ampere_value, 0);
					bl0906_read_register(correction_reg);
				}
			}
		}
	}
	bl0906_update_energy(TYPE_CURRENT, measurement_value.current);

	DBG_BL0906_SEND_STR("\n*************************");
	DBG_BL0906_SEND_STR("\n __CURRENT__:");
	DBG_BL0906_SEND_BYTE(index);
	DBG_BL0906_SEND_STR(", ");
	DBG_BL0906_SEND_INT((uint16_t)measurement_value.current);
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
	switch(cmd_is_running.id_register) {
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
	measurement_value.voltage = \
			(float)data * Vref * (Rf + Rv) / (13162*Rv*Gain_v*1000);
	bl0906_update_energy(TYPE_VOLTAGE, measurement_value.voltage);
	DBG_BL0906_SEND_STR("\n __VOLTAGE__:");
	DBG_BL0906_SEND_INT((uint16_t)measurement_value.voltage);
	DBG_BL0906_SEND_STR(", ");
	DBG_Bl0906_SEND_FLOAT(measurement_value.voltage);
}

/**
 * @func    bl0906_handle_active_power_rsp
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_handle_active_power_rsp(uint8_t* par, uint8_t par_len)
{
	int32_t data = array_to_u24(par);
	if (data < 0) {
		DBG_BL0906_SEND_STR("\n Power Unused");
		return;
	}
	measurement_value.active_power =
		(float)data * Vref * Vref * (Rf + Rv) /
			(40.4125 * Rl*Rv*Gain_i * Gain_v*1000);
	bl0906_update_energy(TYPE_ACTIVE_POWER, measurement_value.active_power);
	DBG_BL0906_SEND_STR("\n __ACTIVE_POWER__:");
	DBG_BL0906_SEND_INT((uint16_t)measurement_value.active_power);
	DBG_BL0906_SEND_STR(", ");
	DBG_Bl0906_SEND_FLOAT(measurement_value.active_power);
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
	measurement_value.temperature =  \
			(data - 64)*12.5/59-40;
	bl0906_update_energy(TYPE_TEMPERATURE, measurement_value.temperature);
}

/**
 * @func    bl0906_handle_temperature_rsp
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_handle_gain_rsp(uint8_t* par, uint8_t par_len)
{
	gain_par.value = array_to_u24(par);
}

/**
 * @func   bl0906_push_wait_reg_rsp_to_fifo
 * @brief
 * @param
 * @retval None
 */
static bool bl0906_push_wait_reg_rsp_to_fifo(uint8_t reg_id)
{
	bl0906_read_cmd_t bl0906_read_cmd;
	bl0906_read_cmd.id_register = reg_id;
	if(FifoPush(&fifo_bl0906_cmd, &bl0906_read_cmd)) {
		return true;
	}
	return false;
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
	if(gain_par.value != GAIN_1_DEFAULT_VALUE) {
		if(clock_time_exceed_ms(gain_par.set_gain_st_t_ms, SET_GAIN_INTERVAL_MS)) {
			bl_0906_set_gain(GAIN_1_DEFAULT_VALUE);
			bl0906_read_register(GAIN_1);
			gain_par.set_gain_st_t_ms = clock_time_ms();
		}
	}
}

/**
 * @func    bl0906_fifo_proc
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_fifo_proc(void)
{
	if(cmd_is_running.id_register != REG_UNKNOWN) {
		if(clock_time_exceed_ms(cmd_is_running.active_st_time, BL0906_READ_TIMEOUT)) {
			cmd_is_running.id_register = REG_UNKNOWN;
		}
	}
	else {
		if (FifoIsEmpty(&fifo_bl0906_cmd) == false){
			if(FifoPop(&fifo_bl0906_cmd, &cmd_is_running)){
				cmd_is_running.active_st_time = clock_time_ms();
					switch(cmd_is_running.id_register) {
					case I1_RMS:
					case I2_RMS:
					case I3_RMS:
						cmd_is_running.p_func = bl0906_handle_current_rsp;
						break;

					case RMSOS_1:
					case RMSOS_2:
					case RMSOS_3:
						cmd_is_running.p_func = bl0906_handle_rmsos_rsp;
						break;

					case V_RMS:
						cmd_is_running.p_func = bl0906_handle_voltage_rsp;
						break;

					case WATT_1:
					case WATT_2:
					case WATT_3:
						cmd_is_running.p_func = bl0906_handle_active_power_rsp;
						break;

					case TPS:
						cmd_is_running.p_func = bl0906_handle_temperature_rsp;
						break;

					case GAIN_1:
						cmd_is_running.p_func = bl0906_handle_gain_rsp;
						break;

					default:
						cmd_is_running.p_func = NULL;
						cmd_is_running.id_register = REG_UNKNOWN;
						break;
				}
				uint8_t tx_data[] = { BL0906_READ_COMMAND, cmd_is_running.id_register };
				bl0906_push_msg(tx_data, sizeof(tx_data));
			}
		}
	}
}

/**
 * @func    bl0906_proc
 * @brief
 * @param
 * @retval  None
 */
void bl0906_proc(void)
{
	bl0906_current_correction_proc();
	bl0906_set_gain_proc();
	bl0906_fifo_proc();
}

/**
 * @func    bl0906_handle_serial_rx_message
 * @brief
 * @param
 * @retval  None
 */
void bl0906_handle_serial_rx_message(uint8_t* buff, uint8_t len)
{
	 uint8_t rx_data[BL0906_RX_LEN];
	 memcpy(rx_data, buff, BL0906_RX_LEN);
	 
	 // ========== Xử lý SYNC (Blocking) Read ==========
	 if (sync_read_waiting && sync_read_register != REG_UNKNOWN) {
		 uint8_t checksum = sync_read_register;
		 for(uint8_t i = 0; i < len-1; i++) {
			 checksum += buff[i];
		 }
		 checksum = checksum^0xFF;
		 if(checksum == buff[len-1]) {
			 // Checksum đúng
			 memcpy(sync_read_response, buff, 3); // Lưu 3 bytes data
			 sync_read_success = true;
			 sync_read_waiting = false;
			 sync_read_register = REG_UNKNOWN;
			 return; // Xử lý xong, return luôn
		 }
	 }
	 
	 // ========== Xử lý ASYNC (FIFO) Read ==========
	 if(cmd_is_running.id_register != REG_UNKNOWN) {
		 if(cmd_is_running.p_func != NULL) {
			 uint8_t checksum = cmd_is_running.id_register ;
			 for(uint8_t i = 0; i < len-1; i++) {
				 checksum += buff[i];
			 }
			 checksum = checksum^0xFF;
			 if(checksum == buff[len-1]) {
				 if(cmd_is_running.p_func != NULL) {
					 cmd_is_running.p_func(rx_data, BL0906_RX_LEN - 1);
				 }
				 cmd_is_running.id_register = REG_UNKNOWN;
				 cmd_is_running.p_func = NULL;
			 }
		 }
	 }
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
 * @func    bl0906_push_msg
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_push_msg(uint8_t *par, uint8_t par_len)
{
	if (p_uart_service != NULL && p_uart_service->getSerial() != NULL) {
		p_uart_service->getSerial()->write(par, par_len);
	}
}

/**
 * @func    bl0940_write_register
 * @brief
 * @param
 * @retval  None
 */
static bool bl0906_write_register(uint8_t address, uint32_t data)
{
	// Remove write protection
	uint8_t tx_wrprot[] = { BL0906_WRITE_COMMAND, 0x9e, 0x55, 0x55, 0x00, 0xb7};
	bl0906_push_msg(tx_wrprot, sizeof(tx_wrprot));
	// Write Register
    uint8_t tx_data[6] = {BL0906_WRITE_COMMAND, address, (uint8_t)(data), (uint8_t)(data >> 8), (uint8_t)(data >> 16)};
    tx_data[5] = _culcCheckSum(&tx_data[1], sizeof(tx_data) - 2, 0, 0);
    bl0906_push_msg(tx_data, sizeof(tx_data));

    // Enable write protection
	uint8_t tx_read_only[] = { BL0906_WRITE_COMMAND, 0x9e, 0x00, 0x00, 0x00, 0x61};
	bl0906_push_msg(tx_read_only, sizeof(tx_read_only));
    return true;
}

/**
 * @func    bl0906_read_register
 * @brief
 * @param
 * @retval  None
 */
static void bl0906_read_register(uint8_t address)
{
	bl0906_push_wait_reg_rsp_to_fifo(address);
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
	bl0906_read_register(I2_RMS);
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
	bl0906_read_register(WATT_2);
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
	if (false == bl0906_write_register(SOFT_RESET, 0x5A5A5A)) {
		DBG_BL0906_SEND_STR("Can not write SOFT_RESET register.");
		return false;
	}
	sleep_ms(500);
	return true;
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
	bl0906_push_msg(tx_data, sizeof(tx_data));
	
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
