/*
 * bl0906.h
 *
 *  Created on:  Nov 12, 2025
 *      Author: DungTranBK
 */

#ifndef BL0906_H_
#define BL0906_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "config_board.h"
#include "utilities.h"

// Forward declaration
class UartService;
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

enum {
	I1_RMS    = 0x0D,
	I2_RMS    = 0x0E,
	I3_RMS    = 0x0F,

	I4_RMS    = 0x10,
	I5_RMS    = 0x13,
	I6_RMS    = 0x14,

	V_RMS     = 0x16,

	WATT_1    = 0x23,
	WATT_2    = 0x24,
	WATT_3    = 0x25,

	WATT_4    = 0x26,
	WATT_5    = 0x29,
	WATT_6    = 0x2A,

	TPS       = 0x5E,

	GAIN_1    = 0x60,

	RMSOS_1    = 0x78,
	RMSOS_2    = 0x79,
	RMSOS_3    = 0x7A,

	USR_WRPROT = 0x9E,
	SOFT_RESET = 0x9F,

	REG_UNKNOWN = 0xFF,
};


typedef enum {
	TYPE_CURRENT       = 0,
	TYPE_VOLTAGE       = 1,
	TYPE_ACTIVE_POWER  = 2,
	TYPE_ACTIVE_ENERGY = 3,
	TYPE_POWER_FACTOR  = 4,
	TYPE_TEMPERATURE   = 5,
} m_type_enum;

#define GAIN_1_DEFAULT_VALUE    0x333300

#define INDEX_UNKNOWN           0xFF

#define BIT_MASK_CURRENT        BIT(0)
#define BIT_MASK_VOLTAGE        BIT(1)
#define BIT_MASK_ACTIVE_POWER   BIT(2)
#define BIT_MASK_ACTIVE_ENERGY  BIT(3)
#define BIT_MASK_POWER_FACTOR   BIT(4)
#define BIT_MASK_TEMPERATURE    BIT(5)
#define BIT_MASK_MAX            BIT_MASK_TEMPERATURE

#define BL0904_MEASUREMENT_MAX  6


typedef struct {
	float current;
	float voltage;
	float active_power;
	float active_energy;
	float power_factor;
	float temperature;
}measurement_value_t;

typedef void (*p_func_handle)(uint8_t* par, uint8_t par_len);

typedef struct {
	uint8_t   id_register;
    uint8_t   header;
    uint32_t  active_st_time;
    p_func_handle p_func;
}bl0906_read_cmd_t;

#define BL0906_READ_COMMAND   0x35
#define BL0906_WRITE_COMMAND  0xCA


#define CURRENT_CORRECTION_RETRY_INTERVAl_MS  TIMER_500MS

#define CURRENT_CORRECTION_TIMEOUT_MS         (CURRENT_CORRECTION_RETRY_INTERVAl_MS*4)+100
#define CURRENT_OFFSET_MA_MAX                 30

typedef struct {
	bool complete_flag[NUMBER_RL];
	uint32_t  rmsos[NUMBER_RL];
	uint32_t  start_time_ms;
	uint32_t  retry_start_time_ms;
	bool is_running;
}current_correction_par_t;


#define SET_GAIN_INTERVAL_MS                 TIMER_5S

typedef struct {
	uint32_t value;
	uint32_t set_gain_st_t_ms;
}gain_par_t;

#define BL0906_RX_LEN              4

#define BL0906_READ_TIMEOUT        100  // milliseconds
#define BL0906_BUF_CMD_SIZE        16
#define POWER_MEASURE_INTERVAL     TIMER_5S


typedef void (*typeBl0906_handle_update_energy)(m_type_enum type, float value);


/******************************************************************************/
/*                             EXPORT FUNCTIONS                               */
/******************************************************************************/

// ========== ASYNC (FIFO) Functions - Giữ nguyên ==========
void bl0906_init(typeBl0906_handle_update_energy func, UartService* uart_service);
void bl0906_proc(void);
bool bl0906_is_correction_complete_or_timeout(void);
void bl0906_handle_serial_rx_message(uint8_t* buff, uint8_t len);
void bl0906_send_get_current(void);
void bl0906_get_voltage(void);
void bl0906_get_active_power(void);
void bl0906_get_active_energy(void);
void bl0906_get_power_factor(void);
void bl0906_get_temperature(void);
void bl0906_measurenment_start(uint16_t m_mask);

// ========== SYNC (Blocking) Functions - Mới thêm ==========
bool bl0906_read_register_sync(uint8_t address, uint8_t* rx_data, uint32_t timeout_ms);
uint32_t bl0906_read_register_sync_u24(uint8_t address, uint32_t timeout_ms);

#endif /* BL0906_H_ */
