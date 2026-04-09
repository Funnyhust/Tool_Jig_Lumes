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
 * File name: process.cpp
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
#include "process.h"
#include "../config.h"
#include "../services/bl0906/bl0906.h"
#include "../services/eeprom_at24c02/at24c02.h"
#include "../services/relay/relay_service.h"
#include "../services/uart/uart_service.h"
#include "../services/write_memory/write_memory.h"
#include "../services/zero_detect/zero_detect.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdio.h>

#define LOW_THRESHOLD 850
#define HIGH_THRESHOLD 1150


// Khai báo hàm delay có blink LED từ main.cpp

// Note hardware
// M2 SCL
// M3 SDA

extern void delayWithBlink(uint32_t ms);

// 4 uart bl0906, 1 uart debug
// Khởi tạo trong hàm để tránh lỗi undefined reference
static UartService *uartBl0906[4] = {NULL, NULL, NULL, NULL};
static UartService *uartDebug = NULL;

// Lưu giá trị đo được cho từng kênh BL0906
static measurement_value_t channel_measurements[4];

uint32_t voltage_sum_uv[4] = {0, 0, 0, 0};
uint32_t current_sum_ua[4][3] = {0, 0, 0, 0};
uint32_t power_sum_uw[4][3] = {0, 0, 0, 0};

// Đếm số lần đo được cho từng kênh BL0906
uint8_t voltage_calib_count[4] = {0, 0, 0, 0};
uint8_t current_calib_count[4][3] = {
    {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
uint8_t power_calib_count[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

uint32_t voltage_calib_value[4] = {0, 0, 0, 0};
// đơn vị uV 4 channel, 3 current
uint32_t current_calib_value[4][3] = {
    {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
uint32_t power_calib_value[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// hệ số calibration
uint16_t kVoltage[4];
uint16_t kCurrent[4][3];
uint16_t kPower[4][3];

uint8_t eeprom_value[4][16];

// Giá trị ngưỡng đọc từ EEPROM (đơn vị như comment trong process.h)
uint32_t VOLTAGE_THRESHOLD_VALUE[4];
uint32_t CURRENT_THRESHOLD_VALUE[4][3];
uint32_t POWER_THRESHOLD_VALUE[4][3];

static void process_calibrate() {

  for (int i = 0; i < 4; i++) {
    if (voltage_calib_count[i] > 0) {
      voltage_calib_value[i] = voltage_sum_uv[i] / voltage_calib_count[i];
      if (voltage_calib_value[i] > 0) {
        kVoltage[i] =
            ((1000 * VOLTAGE_THRESHOLD_VALUE[i]) / voltage_calib_value[i]);
      } else {
        kVoltage[i] = 0;
      }
    }
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (current_calib_count[i][j] > 0) {
        current_calib_value[i][j] =
            current_sum_ua[i][j] / current_calib_count[i][j];
        if (current_calib_value[i][j] > 0) {
          kCurrent[i][j] = ((1000 * CURRENT_THRESHOLD_VALUE[i][j]) /
                            current_calib_value[i][j]);
        } else {
          kCurrent[i][j] = 0;
        }
      }
    }
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (power_calib_count[i][j] > 0) {
        power_calib_value[i][j] = power_sum_uw[i][j] / power_calib_count[i][j];
        if (power_calib_value[i][j] > 0) {
          kPower[i][j] =
              ((1000 * POWER_THRESHOLD_VALUE[i][j]) / power_calib_value[i][j]);
        } else {
          kPower[i][j] = 0;
        }
      }
    }
  }
}

// biến lưu kết quả check từng ralay
static bool check_relay_result[4][3];

// Kết quả độc lập từng state cho 4 kênh (true = PASS)
// Các state chạy đầy đủ, không block nhau, tổng hợp ở STATE 6
static bool result_zcd[4]   = {false, false, false, false}; // STATE 1: Zero Detect
static bool result_calib[4] = {false, false, false, false}; // STATE 3: Calibration
// STATE 4: write_eeprom_success[4] (đã có)
// STATE 5: check_relay_result[4][3] (đã có)


void prepare_wrire_eeprom_value(void) {
  for (int i = 0; i < 4; i++) {
    eeprom_value[i][0] = kVoltage[i];
    eeprom_value[i][1] = kVoltage[i] >> 8;
    for (int j = 0; j < 3; j++) {
      eeprom_value[i][j * 2 + 2] = kCurrent[i][j];
      eeprom_value[i][j * 2 + 2 + 1] = kCurrent[i][j] >> 8;
    }
    for (int j = 0; j < 3; j++) {
      eeprom_value[i][j * 2 + 8] = kPower[i][j];
      eeprom_value[i][j * 2 + 8 + 1] = kPower[i][j] >> 8;
    }
    // 2 byte cuối bằng 0
    eeprom_value[i][14] = 0;
    eeprom_value[i][15] = 0;
  }
}

static uint8_t page_to_write[8];
static uint8_t block_to_read[14];
static bool write_eeprom_success[4] = {false, false, false, false};
void write_eeprom_value(void) {
  for (int ch = 0; ch < 4; ch++) {
    at24c02_init(ch + 1);
    delayWithBlink(10);
    for (int j = 0; j < 3; j++) { // retry max 3 times

      // Ghi page địa chỉ 0x00 - 0x07
      for (int k = 0; k < 8; k++) {
#if EEPROM_TEST_ENABLE
        page_to_write[k] = 0x55 + (millis() % 15);
#else
        page_to_write[k] = eeprom_value[ch][k];
#endif
      }
      if (!at24c02_write_block(0x00, page_to_write, 8)) {
      }
      delayWithBlink(50);
      // Ghi page địa chỉ 0x08 - 0x0F
      for (int k = 0; k < 8; k++) {
#if EEPROM_TEST_ENABLE
        page_to_write[k] = 0x55 + (millis() % 15);
#else
        page_to_write[k] = eeprom_value[ch][k + 8];
#endif
      }
      if (!at24c02_write_block(0x08, page_to_write, 8)) {
      }
      delayWithBlink(50);
      // Đọc page địa chỉ 0x00 - 0x0D
      if (!at24c02_read_block(0x00, block_to_read, 14)) {
      }

      // Kiểm tra giá trị đọc được có giống với giá trị ghi vào không
      bool ok = true;
      for (int k = 0; k < 14; k++) {
        // Note: With EEPROM_TEST_ENABLE, this comparison might fail if logic is
        // not consistent
        if (block_to_read[k] != eeprom_value[ch][k]) {
          ok = false;
          break;
        }
      }
      if (ok) {
        // Nếu ghi thành công thì không retry nữa
        write_eeprom_success[ch] = true;
        break;
      }
    }
  }
}

void start_write_eeprom_value(void) {
  process_calibrate();
  prepare_wrire_eeprom_value();
  write_eeprom_value();
}

// Kết quả zero detect cho 4 kênh
bool zero_detect_result[4] = {true, true, true, true};

void check_channel_pass(int channel) {
  // kiểm tra theo giá trị calib
  if ((kVoltage[channel] < LOW_THRESHOLD) ||
      (kVoltage[channel] > HIGH_THRESHOLD)) {
    channel_measurements[channel].voltage_ok = false;
  } else {
    channel_measurements[channel].voltage_ok = true;
  }
  for(int j=0; j<3; j++) {
    if ((kCurrent[channel][j] < LOW_THRESHOLD) || (kCurrent[channel][j] > HIGH_THRESHOLD)) {
        channel_measurements[channel].current_ok[j] = false;
    } else {
        channel_measurements[channel].current_ok[j] = true;
    }

    if ((kPower[channel][j] < LOW_THRESHOLD) || (kPower[channel][j] > HIGH_THRESHOLD)) {
        channel_measurements[channel].power_ok[j] = false;
    } else {
        channel_measurements[channel].power_ok[j] = true;
    }
  }
}

void debug_init(void) {
  UART_DEBUG.begin(115200);
}

// Khởi tạo bl0906
void process_init(void) {
  // Đọc các giá trị threshold từ EEPROM (Flash của STM32) khi khởi động
  // Nếu có dữ liệu trong EEPROM thì sẽ ghi đè lên giá trị mặc định
  if (read_all_eeprom_channels(VOLTAGE_THRESHOLD_VALUE, CURRENT_THRESHOLD_VALUE,
                               POWER_THRESHOLD_VALUE)) {
  } else {
  }

  // STM32F103VE có 5 UART: Serial1, Serial2, Serial3, Serial4, Serial5
  // Serial1 → Debug
  // Serial2, Serial3, Serial4, Serial5 → 4 kênh BL0906 (channel 0, 1, 2, 3)
  // Serial2 cho BL0906 kênh 0
  if (true) { // Serial2 luôn có trên STM32F103VE
    UART_BL0906_1.begin(19200);
    uartBl0906[0] = new UartService(&UART_BL0906_1, "BL0906_1");
  }

  // Serial3 cho BL0906 kênh 1
  UART_BL0906_2.begin(19200);
  uartBl0906[1] = new UartService(&UART_BL0906_2, "BL0906_2");

  // Serial4 cho BL0906 kênh 2
  UART_BL0906_3.begin(19200);
  uartBl0906[2] = new UartService(&UART_BL0906_3, "BL0906_3");

  // Serial5 cho BL0906 kênh 3
  UART_BL0906_4.begin(19200);
  uartBl0906[3] = new UartService(&UART_BL0906_4, "BL0906_4");

  // Khởi tạo bl0906 cho các UART đã khởi tạo
  for (int i = 0; i < 4; i++) {
    if (uartBl0906[i] != NULL) {
      bl0906_init(NULL, uartBl0906[i]);
    }
  }

  // Set gain cho từng kênh (phải set lại UART trước mỗi lần vì p_uart_service
  // bị ghi đè)
  for (int i = 0; i < 4; i++) {
    if (uartBl0906[i] != NULL) {
      // QUAN TRỌNG: Set lại UART và channel trước khi gọi bl0906_proc()
      // vì p_uart_service bị ghi đè khi init các kênh sau
      bl0906_set_uart(uartBl0906[i]);
      bl0906_set_channel(i);
      bl0906_proc(); // Kiểm tra và set gain cho kênh này
      delayWithBlink(100);
    }
  }
}
// Khai báo extern relayService từ main.cpp
extern RelayService relayService;

// Helper function: Đọc từ một BL0906 cụ thể
static void read_bl0906_channel(int channel, UartService *uart) {
  if (uart == NULL) {
    return;
  }

  // Set UART và channel cho bl0906
  bl0906_set_uart(uart);
  // Đảm bảo UART được set đúng (quan trọng cho kênh 0)
  if (uart == NULL || uart->getSerial() == NULL) {
    return;
  }
  // Flush UART để đảm bảo buffer sạch trước khi set gain (quan trọng cho kênh
  // 0)
  uart->getSerial()->flush();
  bl0906_set_channel(
      channel); // Set channel trước khi đọc (sẽ tự động reset giá trị)

  // Kiểm tra và set gain cho kênh này (đảm bảo gain đúng trước khi đọc)
  bl0906_proc();
  // Delay sau khi set gain để BL0906 có thời gian cập nhật giá trị
  delay(5);

  // Đọc giá trị
  bl0906_send_get_current();
  bl0906_proc();
  bl0906_get_voltage();
  bl0906_proc();
  bl0906_get_active_power();

  // Lưu giá trị vào mảng của kênh này ngay sau khi đọc giá trị đơn vị uV, pA,
  // uW
  channel_measurements[channel] = bl0906_get_all_measurements();
  if (channel_measurements[channel].voltage > 0) {
    voltage_sum_uv[channel] += channel_measurements[channel].voltage * 1000;
    voltage_calib_count[channel]++;
  }
  if (channel_measurements[channel].current[0] > 0) {
    current_sum_ua[channel][0] +=
        channel_measurements[channel].current[0] * 1000;
    current_calib_count[channel][0]++;
  }
  if (channel_measurements[channel].current[1] > 0) {
    current_sum_ua[channel][1] +=
        channel_measurements[channel].current[1] * 1000;
    current_calib_count[channel][1]++;
  }
  if (channel_measurements[channel].current[2] > 0) {
    current_sum_ua[channel][2] +=
        channel_measurements[channel].current[2] * 1000;
    current_calib_count[channel][2]++;
  }
  if (channel_measurements[channel].active_power[0] > 0) {
    power_sum_uw[channel][0] +=
        channel_measurements[channel].active_power[0] * 1000;
    power_calib_count[channel][0]++;
  }
  if (channel_measurements[channel].active_power[1] > 0) {
    power_sum_uw[channel][1] +=
        channel_measurements[channel].active_power[1] * 1000;
    power_calib_count[channel][1]++;
  }
  if (channel_measurements[channel].active_power[2] > 0) {
    power_sum_uw[channel][2] +=
        channel_measurements[channel].active_power[2] * 1000;
    power_calib_count[channel][2]++;
  }
}

// Khai báo biến điều khiển LED từ main.cpp
extern volatile bool ledBlinkEnable;

//===========================================//
//=========== LOGGING TELEMETRY =============//
//===========================================//

#define LOG_HEADER       0xAA
#define LOG_CMD_START    0xA0
#define LOG_CMD_END      0xA1
#define LOG_CMD_SUMMARY  0xA2
#define LOG_CMD_CH1      0x11
#define LOG_CMD_CH2      0x12
#define LOG_CMD_CH3      0x13
#define LOG_CMD_CH4      0x14

static void send_test_event(uint8_t cmd) {
  uint8_t pkt[5];
  pkt[0] = LOG_HEADER;
  pkt[1] = cmd;
  pkt[2] = 0x00;
  pkt[3] = 0x00;
  pkt[4] = (LOG_HEADER + cmd) & 0xFF;
  UART_DEBUG.write(pkt, 5);
}

static void broadcast_channel_data(int ch) {
  // Single-channel payload (24 bytes total):
  if (ch < 0 || ch > 3) return;
  uint8_t buf[24];
  memset(buf, 0, sizeof(buf));
  buf[0] = LOG_HEADER;
  buf[1] = LOG_CMD_CH1 + ch;

  measurement_value_t &m = channel_measurements[ch];

  // Voltage
  uint16_t v = (uint16_t)(m.voltage * 100.0f);
  buf[2] = (v >> 8) & 0xFF;
  buf[3] = v & 0xFF;

  // Currents & Powers
  for (int r = 0; r < 3; r++) {
    uint16_t cur = (uint16_t)(m.current[r] * 10.0f);
    buf[4 + r*2]     = (cur >> 8) & 0xFF;
    buf[4 + r*2 + 1] = cur & 0xFF;
    
    uint16_t pwr = (uint16_t)(m.active_power[r] * 100.0f);
    buf[10 + r*2]     = (pwr >> 8) & 0xFF;
    buf[10 + r*2 + 1] = pwr & 0xFF;
  }

  // Gain
  uint32_t g = m.gain;
  buf[16] = (g >> 24) & 0xFF;
  buf[17] = (g >> 16) & 0xFF;
  buf[18] = (g >> 8) & 0xFF;
  buf[19] = g & 0xFF;

  // Relay mask
  uint16_t mask = 0;
  for (int i = 0; i < 12; i++) {
    if (relayService.isOn(i)) mask |= (1 << i);
  }
  buf[20] = (mask >> 8) & 0xFF;
  buf[21] = mask & 0xFF;

  // Checksum
  uint8_t cs = 0;
  for (int i = 0; i < 22; i++) cs += buf[i];
  buf[22] = cs;

  UART_DEBUG.write(buf, 24);
}

static void broadcast_summary_frame(void) {
  // [14:26] relay_ok (12 bytes)
  // [26:34] zcd_counts (8 bytes: 4x uint16)
  // [34:42] k_u (8 bytes: 4x uint16)
  // [42:66] k_i (24 bytes: 12x uint16)
  // [66:90] k_p (24 bytes: 12x uint16)
  // [90:94] cnt_u (4 bytes: 4x uint8)
  // [94:106] cnt_i (12 bytes: 12x uint8)
  // [106]    Checksum
  // Total: 108 bytes

  uint8_t buf[108];
  memset(buf, 0, sizeof(buf));
  buf[0] = LOG_HEADER;
  buf[1] = LOG_CMD_SUMMARY;

  // Fill status bytes
  for (int i = 0; i < 4; i++) {
    buf[2 + i] = (uint8_t)result_zcd[i];
    buf[6 + i] = (uint8_t)result_calib[i];
    buf[10 +i] = (uint8_t)write_eeprom_success[i];
    
    for (int j = 0; j < 3; j++) {
      buf[14 + i*3 + j] = (uint8_t)check_relay_result[i][j];
    }
  }

  // ZCD Counts (uint16 big-endian)
  for (int i = 0; i < 4; i++) {
    uint16_t count = (uint16_t)zero_detect_get_count(i);
    buf[26 + i*2]     = (count >> 8) & 0xFF;
    buf[26 + i*2 + 1] = count & 0xFF;
  }

  // K_U
  for (int i = 0; i < 4; i++) {
    uint16_t ku = (uint16_t)kVoltage[i];
    buf[34 + i*2]     = (ku >> 8) & 0xFF;
    buf[34 + i*2 + 1] = ku & 0xFF;
  }

  // K_I & K_P (12 each)
  for (int i = 0; i < 12; i++) {
    int ch_idx = i / 3;
    int rel_idx = i % 3;
    uint16_t ki = (uint16_t)kCurrent[ch_idx][rel_idx];
    uint16_t kp = (uint16_t)kPower[ch_idx][rel_idx];
    buf[42 + i*2]     = (ki >> 8) & 0xFF;
    buf[42 + i*2 + 1] = ki & 0xFF;
    buf[66 + i*2]     = (kp >> 8) & 0xFF;
    buf[66 + i*2 + 1] = kp & 0xFF;
  }

  // Counts (uint8)
  for (int i = 0; i < 4; i++) buf[90 + i] = (uint8_t)voltage_calib_count[i];
  for (int i = 0; i < 12; i++) {
     int ch_idx = i / 3;
     int rel_idx = i % 3;
     buf[94 + i] = (uint8_t)current_calib_count[ch_idx][rel_idx];
  }

  // CS
  uint8_t cs = 0;
  for (int i = 0; i < 107; i++) cs += buf[i];
  buf[107] = cs;

  UART_DEBUG.write(buf, 108);
}

//===========================================//
//==================PROCESS==================//
//===========================================//

void start_process(void) {
  // Gửi sự kiện START cho App Desktop
  send_test_event(LOG_CMD_START);

  // Clear all measurement buffers and summation logic to prevent stale data (per user request)
  memset(channel_measurements, 0, sizeof(channel_measurements));
  memset(voltage_sum_uv, 0, sizeof(voltage_sum_uv));
  memset(current_sum_ua, 0, sizeof(current_sum_ua));
  memset(power_sum_uw, 0, sizeof(power_sum_uw));
  memset(voltage_calib_count, 0, sizeof(voltage_calib_count));
  memset(current_calib_count, 0, sizeof(current_calib_count));
  memset(power_calib_count, 0, sizeof(power_calib_count));
  memset(voltage_calib_value, 0, sizeof(voltage_calib_value));
  memset(current_calib_value, 0, sizeof(current_calib_value));
  memset(power_calib_value, 0, sizeof(power_calib_value));

  // Init check_relay_result
  for(int i=0; i<4; i++){
      for(int j=0; j<3; j++){
          check_relay_result[i][j] = true;
      }
  }

  uint32_t start_time_ms = millis();
  
  // Bật blink LED
  ledBlinkEnable = true;
  delayWithBlink(100);
  //==================STATE 1: ZERO DETECT ALL CHANNELS==================//
  zero_detect_process(); // Measure all 4 channels at once

  // Lưu kết quả ZCD vào biến state (sau khi đo xong)
  for (int i = 0; i < 4; i++) {
    result_zcd[i] = zero_detect_get_result(i);
  }

  // Read measurements for each channel one at a time
  for (int ch = 0; ch < 4; ch++) {
    // Clear ALL measurement buffers before reading this specific channel (per user request)
    // This ensures that 'old' data from previously measured channels doesn't show up in the log.
    memset(channel_measurements, 0, sizeof(channel_measurements));

    
    if (uartBl0906[ch] == NULL) {
      continue;
    }
    
    //==================STEP 1: TURN ON RELAYS FOR THIS CHANNEL==================//
    for(int j=0; j<3; j++) {
      relayService.setRelayState(ch * 3 + j, true);
      delayWithBlink(400);
    }
    //==================STEP 2: PREPARE AND STABILIZE==================//
    // Set UART and channel BEFORE delay to allow background calibration
    bl0906_set_uart(uartBl0906[ch]);
    delay(5); 
    bl0906_set_channel(ch);

    // Optimized stabilization wait: Try to write GAIN during the idle 3s
    uint32_t wait_start = millis();
    
    while (millis() - wait_start < 3000) {
        bl0906_proc();
        delayWithBlink(100); 
    }
    
    // Read this channel 5 times
    for (int j = 0; j < 5; j++) {
      uint32_t calib_start_time_ms = millis();
      
      if (millis() - calib_start_time_ms > 1000) {
        break;
      }
      
      // Read this channel only
      read_bl0906_channel(ch, uartBl0906[ch]);
      broadcast_channel_data(ch); // << Gửi dữ liệu kênh này lên App
      delayWithBlink(2);
      
      if (j < 4) {
        while (millis() - calib_start_time_ms < 500) {
          delayWithBlink(5);
        }
      }
    }
    
    //==================STEP 3: TURN OFF RELAYS FOR THIS CHANNEL==================//
    for(int j=0; j<3; j++) {
      relayService.setRelayState(ch * 3 + j, false);
      delayWithBlink(200);
    }
  }
  
  //==================STATE 3: CALIBRATE ALL CHANNELS==================//
  
  // Calculate K values for all channels
  for (int ch = 0; ch < 4; ch++) {
    
    if (voltage_calib_count[ch] > 0) {
      voltage_calib_value[ch] = voltage_sum_uv[ch] / voltage_calib_count[ch];
      if (voltage_calib_value[ch] > 0) {
        kVoltage[ch] = ((1000 * VOLTAGE_THRESHOLD_VALUE[ch]) / voltage_calib_value[ch]);
      } else {
        kVoltage[ch] = 0;
      }
    }
    
    for (int j = 0; j < 3; j++) {
      if (current_calib_count[ch][j] > 0) {
        current_calib_value[ch][j] = current_sum_ua[ch][j] / current_calib_count[ch][j];
        if (current_calib_value[ch][j] > 0) {
          kCurrent[ch][j] = ((1000 * CURRENT_THRESHOLD_VALUE[ch][j]) / current_calib_value[ch][j]);
        } else {
          kCurrent[ch][j] = 0;
        }
      }
      
      if (power_calib_count[ch][j] > 0) {
        power_calib_value[ch][j] = power_sum_uw[ch][j] / power_calib_count[ch][j];
        if (power_calib_value[ch][j] > 0) {
          kPower[ch][j] = ((1000 * POWER_THRESHOLD_VALUE[ch][j]) / power_calib_value[ch][j]);
        } else {
          kPower[ch][j] = 0;
        }
      }
    }
    
    // Check pass/fail for this channel
    check_channel_pass(ch);

    // Lưu kết quả calib state: pass khi tất cả V, I, P đều trong ngưỡng K
    result_calib[ch] = channel_measurements[ch].voltage_ok
                    && channel_measurements[ch].current_ok[0]
                    && channel_measurements[ch].current_ok[1]
                    && channel_measurements[ch].current_ok[2]
                    && channel_measurements[ch].power_ok[0]
                    && channel_measurements[ch].power_ok[1]
                    && channel_measurements[ch].power_ok[2];
  }
  
  //==================STATE 4: WRITE EEPROM FOR ALL CHANNELS==================//
  
  for (int ch = 0; ch < 4; ch++) {
    // Prepare EEPROM data for this channel
    eeprom_value[ch][0] = kVoltage[ch];
    eeprom_value[ch][1] = kVoltage[ch] >> 8;
    for (int j = 0; j < 3; j++) {
      eeprom_value[ch][j * 2 + 2] = kCurrent[ch][j];
      eeprom_value[ch][j * 2 + 2 + 1] = kCurrent[ch][j] >> 8;
    }
    for (int j = 0; j < 3; j++) {
      eeprom_value[ch][j * 2 + 8] = kPower[ch][j];
      eeprom_value[ch][j * 2 + 8 + 1] = kPower[ch][j] >> 8;
    }
    eeprom_value[ch][14] = 0;
    eeprom_value[ch][15] = 0;
    
    // Write to EEPROM for this channel
    at24c02_init(ch + 1);
    delayWithBlink(10);
    
    for (int j = 0; j < 5; j++) { // retry max 3 times
      
      // Write page 0x00 - 0x07
      for (int k = 0; k < 8; k++) {
        page_to_write[k] = eeprom_value[ch][k];
      }
      if (!at24c02_write_block(0x00, page_to_write, 8)) {
      }
      delayWithBlink(50);
      
      // Write page 0x08 - 0x0F
      for (int k = 0; k < 8; k++) {
        page_to_write[k] = eeprom_value[ch][k + 8];
      }
      if (!at24c02_write_block(0x08, page_to_write, 8)) {
      }
      delayWithBlink(50);
      
      // Read back and verify
      if (!at24c02_read_block(0x00, block_to_read, 14)) {
      }
      
      // Check if read data matches written data
      bool ok = true;
      for (int k = 0; k < 14; k++) {
        if (block_to_read[k] != eeprom_value[ch][k]) {
          ok = false;
          break;
        }
      }
      if (ok) {
        write_eeprom_success[ch] = true;
        break;
      }
    }
  }

   //==================STATE 5: LOOP RELAY (PARALLEL BATCH CHECK)==================//
  // Test Relay 1 of ALL channels simultaneously, then Relay 2, then Relay 3.
  // Đo 3 lần mỗi relay index để tránh false-negative do BL0906 trả sai nhất thời.
  // Chỉ FAIL khi cả 3 lần đo đều thất bại (fail_count == NUM_RELAY_MEASUREMENTS).

  // Clear all buffers before starting state 5 batch check
  memset(channel_measurements, 0, sizeof(channel_measurements));
  const int NUM_RELAY_MEASUREMENTS = 3;

  // Loop through Relay Indices (0 -> 1 -> 2)
  for (int j = 0; j < 3; j++) {
      // Clear buffers before EACH relay category test to ensure fresh data
      memset(channel_measurements, 0, sizeof(channel_measurements));
      
      relayService.turnOffAll();

      // 1. Turn ON Relay 'j' for ALL channels
      for (int i = 0; i < 4; i++) {
           relayService.setRelayState(i * 3 + j, true);
      }
      uint32_t wait_start_2 = millis();
    
      while (millis() - wait_start_2 < 1000) {
          bl0906_proc();
          delayWithBlink(100); 
      }
      // Bộ đếm số lần fail cho từng (channel, relay).
      // Chỉ mark FAIL khi fail_count == NUM_RELAY_MEASUREMENTS (tất cả lần đều fail).
      uint8_t fail_count[4][3] = {
          {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      };

      // 2. Đo NUM_RELAY_MEASUREMENTS lần
      for (int meas = 0; meas < NUM_RELAY_MEASUREMENTS; meas++) {

          for (int m_ch = 0; m_ch < 4; m_ch++) {
              read_bl0906_channel(3 - m_ch, uartBl0906[3 - m_ch]);
              broadcast_channel_data(3 - m_ch); // Gửi riêng từng kênh
          }

          // Tích lũy fail_count cho lần đo này
          for (int ch_chk = 0; ch_chk < 4; ch_chk++) {
              for (int r_chk = 0; r_chk < 3; r_chk++) {
                  bool should_be_on = (r_chk == j);
                  if (should_be_on) {
                      if (channel_measurements[ch_chk].current[r_chk] < 100 ||
                          channel_measurements[ch_chk].active_power[r_chk] < 20) {
                          fail_count[ch_chk][r_chk]++;
                      }
                  } else {
                      // This relay was NOT turned on. It MUST measure near zero.
                      if (channel_measurements[ch_chk].current[r_chk] > 120 ||
                          channel_measurements[ch_chk].active_power[r_chk] > 30) {
                          fail_count[ch_chk][r_chk]++;
                      }
                  }
              }
          }

          // Delay giữa các lần đo (trừ lần cuối)
          if (meas < NUM_RELAY_MEASUREMENTS - 1) {
              delay(650);
          }
      } // end meas loop

      // 3. Verify: Chỉ mark FAIL nếu TẤT CẢ lần đo đều fail
      for (int ch_chk = 0; ch_chk < 4; ch_chk++) {
          for (int r_chk = 0; r_chk < 3; r_chk++) {
              if (fail_count[ch_chk][r_chk] == NUM_RELAY_MEASUREMENTS) {
                  check_relay_result[ch_chk][r_chk] = false; 
              }
          }
      }

      // 4. Turn OFF Relay 'j' for all channels
      for (int i = 0; i < 4; i++) {
           relayService.setRelayState(i * 3 + j, false);
      }
  }

  // Tổng hợp kết quả từ 4 state độc lập
  // PASS = ZCD pass && Calib pass && EEPROM write pass && Relay test pass
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      bool channel_pass = result_zcd[i]
                       && result_calib[i]
                       && write_eeprom_success[i]
                       && check_relay_result[i][j];
      relayService.setRelayState(i * 3 + j, channel_pass);
    }
  }

  // Gửi thông số tổng hợp (ZCD, Calib, EEPROM, Relay) cho App Desktop
  broadcast_summary_frame();

  // Gửi sự kiện END cho App Desktop
  send_test_event(LOG_CMD_END);
}  

