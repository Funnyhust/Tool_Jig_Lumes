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

#if PROCESS_DEBUG_ENABLE
#define PROCESS_UART_DEBUG_PRINT(x) UART_DEBUG.print(x)
#define PROCESS_UART_DEBUG_PRINTLN(x) UART_DEBUG.println(x)
#else
#define PROCESS_UART_DEBUG_PRINT(x)
#define PROCESS_UART_DEBUG_PRINTLN(x)
#endif

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
      PROCESS_UART_DEBUG_PRINT("Retry: ");
      PROCESS_UART_DEBUG_PRINTLN(j + 1);

      // Ghi page địa chỉ 0x00 - 0x07
      for (int k = 0; k < 8; k++) {
#if EEPROM_TEST_ENABLE
        page_to_write[k] = 0x55 + (millis() % 15);
#else
        page_to_write[k] = eeprom_value[ch][k];
#endif
      }
      if (!at24c02_write_block(0x00, page_to_write, 8)) {
        PROCESS_UART_DEBUG_PRINT("CH");
        PROCESS_UART_DEBUG_PRINT(ch + 1);
        PROCESS_UART_DEBUG_PRINTLN(": I2C Write NACK (No Response) at 0x00");
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
        PROCESS_UART_DEBUG_PRINT("CH");
        PROCESS_UART_DEBUG_PRINT(ch + 1);
        PROCESS_UART_DEBUG_PRINTLN(": I2C Write NACK (No Response) at 0x08");
      }
      delayWithBlink(50);
      // Đọc page địa chỉ 0x00 - 0x0D
      if (!at24c02_read_block(0x00, block_to_read, 14)) {
        PROCESS_UART_DEBUG_PRINT("CH");
        PROCESS_UART_DEBUG_PRINT(ch + 1);
        PROCESS_UART_DEBUG_PRINTLN(": I2C Read NACK (No Response)");
      }

      // Kiểm tra giá trị đọc được có giống với giá trị ghi vào không
      bool ok = true;
      for (int k = 0; k < 14; k++) {
        // Note: With EEPROM_TEST_ENABLE, this comparison might fail if logic is
        // not consistent
        if (block_to_read[k] != eeprom_value[ch][k]) {
          ok = false;
          PROCESS_UART_DEBUG_PRINT("CH");
          PROCESS_UART_DEBUG_PRINT(ch + 1);
          PROCESS_UART_DEBUG_PRINT(" Data Mismatch Index: ");
          PROCESS_UART_DEBUG_PRINT(k);
          PROCESS_UART_DEBUG_PRINT(" Exp: ");
          PROCESS_UART_DEBUG_PRINT((int)eeprom_value[ch][k]);
          PROCESS_UART_DEBUG_PRINT(" Act: ");
          PROCESS_UART_DEBUG_PRINTLN((int)block_to_read[k]);
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
  PROCESS_UART_DEBUG_PRINT("He so calib: ");
  PROCESS_UART_DEBUG_PRINT(" channel: ");
  PROCESS_UART_DEBUG_PRINT(channel + 1);
  PROCESS_UART_DEBUG_PRINT("   ||kV: ");
  PROCESS_UART_DEBUG_PRINT(kVoltage[channel]);
  PROCESS_UART_DEBUG_PRINT("   ||kI0: ");
  PROCESS_UART_DEBUG_PRINT(kCurrent[channel][0]);
  PROCESS_UART_DEBUG_PRINT("   ||kI1: ");
  PROCESS_UART_DEBUG_PRINT(kCurrent[channel][1]);
  PROCESS_UART_DEBUG_PRINT("   ||kI2: ");
  PROCESS_UART_DEBUG_PRINT(kCurrent[channel][2]);
  PROCESS_UART_DEBUG_PRINT("   ||kP0: ");
  PROCESS_UART_DEBUG_PRINT(kPower[channel][0]);
  PROCESS_UART_DEBUG_PRINT("   ||kP1: ");
  PROCESS_UART_DEBUG_PRINT(kPower[channel][1]);
  PROCESS_UART_DEBUG_PRINT("   ||kP2: ");
  PROCESS_UART_DEBUG_PRINTLN(kPower[channel][2]);
}

void debug_init(void) {
  UART_DEBUG.begin(115200);
  uartDebug = new UartService(&UART_DEBUG, "DEBUG");
}

// Khởi tạo bl0906
void process_init(void) {
  // Đọc các giá trị threshold từ EEPROM (Flash của STM32) khi khởi động
  // Nếu có dữ liệu trong EEPROM thì sẽ ghi đè lên giá trị mặc định
  if (read_all_eeprom_channels(VOLTAGE_THRESHOLD_VALUE, CURRENT_THRESHOLD_VALUE,
                               POWER_THRESHOLD_VALUE)) {
    PROCESS_UART_DEBUG_PRINTLN("Loaded thresholds from Flash memory");
  } else {
    PROCESS_UART_DEBUG_PRINTLN("Using default thresholds (no data in Flash)");
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
    PROCESS_UART_DEBUG_PRINT("ERROR: Channel ");
    PROCESS_UART_DEBUG_PRINT(channel);
    PROCESS_UART_DEBUG_PRINTLN(" UART is NULL!");
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
  bl0906_get_voltage();
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
  uint32_t t9 = millis();
  PROCESS_UART_DEBUG_PRINT("Channel: ");
  PROCESS_UART_DEBUG_PRINT(channel + 1);
  PROCESS_UART_DEBUG_PRINT("   ||Voltage: ");
  PROCESS_UART_DEBUG_PRINT(channel_measurements[channel].voltage);
  PROCESS_UART_DEBUG_PRINT("   ||Current 0: ");
  PROCESS_UART_DEBUG_PRINT(channel_measurements[channel].current[0]);
  PROCESS_UART_DEBUG_PRINT("   ||Current 1: ");
  PROCESS_UART_DEBUG_PRINT(channel_measurements[channel].current[1]);
  PROCESS_UART_DEBUG_PRINT("   ||Current 2: ");
  PROCESS_UART_DEBUG_PRINT(channel_measurements[channel].current[2]);
  PROCESS_UART_DEBUG_PRINT("   ||Power 0: ");
  PROCESS_UART_DEBUG_PRINT(channel_measurements[channel].active_power[0]);
  PROCESS_UART_DEBUG_PRINT("   ||Power 1: ");
  PROCESS_UART_DEBUG_PRINT(channel_measurements[channel].active_power[1]);
  PROCESS_UART_DEBUG_PRINT("   ||Power 2: ");
  PROCESS_UART_DEBUG_PRINTLN(channel_measurements[channel].active_power[2]);
}

// Khai báo biến điều khiển LED từ main.cpp
extern volatile bool ledBlinkEnable;

//===========================================//
//==================PROCESS==================//
//===========================================//

void start_process(void) {
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
  PROCESS_UART_DEBUG_PRINTLN("STATE 1: Zero Detect for all channels simultaneously");
  zero_detect_process(); // Measure all 4 channels at once

  //==================STATE 2: READ ALL CHANNELS SEQUENTIALLY==================//
  // Read measurements for each channel one at a time
  for (int ch = 0; ch < 4; ch++) {
    PROCESS_UART_DEBUG_PRINT("========== Reading Channel: ");
    PROCESS_UART_DEBUG_PRINT(ch + 1);
    PROCESS_UART_DEBUG_PRINTLN(" ==========");
    
    if (uartBl0906[ch] == NULL) {
      PROCESS_UART_DEBUG_PRINT("Channel ");
      PROCESS_UART_DEBUG_PRINT(ch + 1);
      PROCESS_UART_DEBUG_PRINTLN(" UART is NULL, skipping...");
      continue;
    }
    
    //==================STEP 1: TURN ON RELAYS FOR THIS CHANNEL==================//
    PROCESS_UART_DEBUG_PRINT("STEP 1: Turning ON relays for Channel ");
    PROCESS_UART_DEBUG_PRINTLN(ch + 1);
    for(int j=0; j<3; j++) {
      relayService.setRelayState(ch * 3 + j, true);
    }
    delayWithBlink(1500); // Delay to stabilize
    
    //==================STEP 2: READ BL0906 FOR THIS CHANNEL==================//
    PROCESS_UART_DEBUG_PRINT("STEP 2: Reading BL0906 for Channel ");
    PROCESS_UART_DEBUG_PRINTLN(ch + 1);
    
    // Set UART và channel
    bl0906_set_uart(uartBl0906[ch]);
    delay(5); // Đợi UART được set xong
    bl0906_set_channel(ch);
    
    // Print threshold values
    PROCESS_UART_DEBUG_PRINT("Gia tri tai chuan ");
    PROCESS_UART_DEBUG_PRINT(ch + 1);
    PROCESS_UART_DEBUG_PRINT("   ||Voltage: ");
    PROCESS_UART_DEBUG_PRINT((float)VOLTAGE_THRESHOLD_VALUE[ch] / 1000);
    PROCESS_UART_DEBUG_PRINT("   ||Current 0: ");
    PROCESS_UART_DEBUG_PRINT((float)CURRENT_THRESHOLD_VALUE[ch][0] / 1000);
    PROCESS_UART_DEBUG_PRINT("   ||Current 1: ");
    PROCESS_UART_DEBUG_PRINT((float)CURRENT_THRESHOLD_VALUE[ch][1] / 1000);
    PROCESS_UART_DEBUG_PRINT("   ||Current 2: ");
    PROCESS_UART_DEBUG_PRINT((float)CURRENT_THRESHOLD_VALUE[ch][2] / 1000);
    PROCESS_UART_DEBUG_PRINT("   ||Power 0: ");
    PROCESS_UART_DEBUG_PRINT((float)POWER_THRESHOLD_VALUE[ch][0] / 1000);
    PROCESS_UART_DEBUG_PRINT("   ||Power 1: ");
    PROCESS_UART_DEBUG_PRINT((float)POWER_THRESHOLD_VALUE[ch][1] / 1000);
    PROCESS_UART_DEBUG_PRINT("   ||Power 2: ");
    PROCESS_UART_DEBUG_PRINTLN((float)POWER_THRESHOLD_VALUE[ch][2] / 1000);
    
    // Read this channel 5 times
    for (int j = 0; j < 5; j++) {
      PROCESS_UART_DEBUG_PRINT("Lan doc: ");
      PROCESS_UART_DEBUG_PRINTLN(j + 1);
      uint32_t calib_start_time_ms = millis();
      
      if (millis() - calib_start_time_ms > 1000) {
        break;
      }
      
      // Read this channel only
      read_bl0906_channel(ch, uartBl0906[ch]);
      delayWithBlink(2);
      
      if (j < 4) {
        while (millis() - calib_start_time_ms < 600) {
          delayWithBlink(5);
        }
      }
    }
    
    //==================STEP 3: TURN OFF RELAYS FOR THIS CHANNEL==================//
    PROCESS_UART_DEBUG_PRINT("STEP 3: Turning OFF relays for Channel ");
    PROCESS_UART_DEBUG_PRINTLN(ch + 1);
    for(int j=0; j<3; j++) {
      relayService.setRelayState(ch * 3 + j, false);
    }
    delayWithBlink(100);
  }
  
  //==================STATE 3: CALIBRATE ALL CHANNELS==================//
  PROCESS_UART_DEBUG_PRINTLN("STATE 3: Calibrating all channels");
  
  // Calculate K values for all channels
  for (int ch = 0; ch < 4; ch++) {
    PROCESS_UART_DEBUG_PRINT("Calibrating Channel ");
    PROCESS_UART_DEBUG_PRINTLN(ch + 1);
    
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
  }
  
  //==================STATE 4: WRITE EEPROM FOR ALL CHANNELS==================//
  PROCESS_UART_DEBUG_PRINTLN("STATE 4: Writing EEPROM for all channels");
  
  for (int ch = 0; ch < 4; ch++) {
    PROCESS_UART_DEBUG_PRINT("Writing EEPROM for Channel ");
    PROCESS_UART_DEBUG_PRINTLN(ch + 1);
    
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
    
    for (int j = 0; j < 3; j++) { // retry max 3 times
      PROCESS_UART_DEBUG_PRINT("Retry: ");
      PROCESS_UART_DEBUG_PRINTLN(j + 1);
      
      // Write page 0x00 - 0x07
      for (int k = 0; k < 8; k++) {
        page_to_write[k] = eeprom_value[ch][k];
      }
      if (!at24c02_write_block(0x00, page_to_write, 8)) {
        PROCESS_UART_DEBUG_PRINT("CH");
        PROCESS_UART_DEBUG_PRINT(ch + 1);
        PROCESS_UART_DEBUG_PRINTLN(": I2C Write NACK (No Response) at 0x00");
      }
      delayWithBlink(50);
      
      // Write page 0x08 - 0x0F
      for (int k = 0; k < 8; k++) {
        page_to_write[k] = eeprom_value[ch][k + 8];
      }
      if (!at24c02_write_block(0x08, page_to_write, 8)) {
        PROCESS_UART_DEBUG_PRINT("CH");
        PROCESS_UART_DEBUG_PRINT(ch + 1);
        PROCESS_UART_DEBUG_PRINTLN(": I2C Write NACK (No Response) at 0x08");
      }
      delayWithBlink(50);
      
      // Read back and verify
      if (!at24c02_read_block(0x00, block_to_read, 14)) {
        PROCESS_UART_DEBUG_PRINT("CH");
        PROCESS_UART_DEBUG_PRINT(ch + 1);
        PROCESS_UART_DEBUG_PRINTLN(": I2C Read NACK (No Response)");
      }
      
      // Check if read data matches written data
      bool ok = true;
      for (int k = 0; k < 14; k++) {
        if (block_to_read[k] != eeprom_value[ch][k]) {
          ok = false;
          PROCESS_UART_DEBUG_PRINT("CH");
          PROCESS_UART_DEBUG_PRINT(ch + 1);
          PROCESS_UART_DEBUG_PRINT(" Data Mismatch Index: ");
          PROCESS_UART_DEBUG_PRINT(k);
          PROCESS_UART_DEBUG_PRINT(" Exp: ");
          PROCESS_UART_DEBUG_PRINT((int)eeprom_value[ch][k]);
          PROCESS_UART_DEBUG_PRINT(" Act: ");
          PROCESS_UART_DEBUG_PRINTLN((int)block_to_read[k]);
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
  PROCESS_UART_DEBUG_PRINTLN("STATE 5: PARALLEL RELAY TEST & LEAKAGE CHECK");

  const int NUM_RELAY_MEASUREMENTS = 3;

  // Loop through Relay Indices (0 -> 1 -> 2)
  for (int j = 0; j < 3; j++) {
      PROCESS_UART_DEBUG_PRINT("Testing Relay Index: ");
      PROCESS_UART_DEBUG_PRINTLN(j + 1);

      // 1. Turn ON Relay 'j' for all valid channels
      for (int i = 0; i < 4; i++) {
           if ( (LOW_THRESHOLD<=kVoltage[i]<=HIGH_THRESHOLD) &
            (LOW_THRESHOLD<=kCurrent[i][j]<=HIGH_THRESHOLD) &
            (LOW_THRESHOLD<=kPower[i][j]<=HIGH_THRESHOLD) &
            write_eeprom_success[i] & zero_detect_get_result(i)) {
               relayService.setRelayState(i * 3 + j, true);
           }
      }
      delay(1000); // Stabilize load

      // Bộ đếm số lần fail cho từng (channel, relay).
      // Chỉ mark FAIL khi fail_count == NUM_RELAY_MEASUREMENTS (tất cả lần đều fail).
      uint8_t fail_count[4][3] = {
          {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      };

      // 2. Đo NUM_RELAY_MEASUREMENTS lần
      for (int meas = 0; meas < NUM_RELAY_MEASUREMENTS; meas++) {
          PROCESS_UART_DEBUG_PRINT("  Measurement round: ");
          PROCESS_UART_DEBUG_PRINTLN(meas + 1);

          // Measure ALL channels
          for (int m_ch = 0; m_ch < 4; m_ch++) {
              read_bl0906_channel(3 - m_ch, uartBl0906[3 - m_ch]);
          }

          // Tích lũy fail_count cho lần đo này
          for (int ch_chk = 0; ch_chk < 4; ch_chk++) {
              for (int r_chk = 0; r_chk < 3; r_chk++) {
                  bool should_be_on = (r_chk == j);
                  if (should_be_on) {
                      if (channel_measurements[ch_chk].current[r_chk] < 100 ||
                          channel_measurements[ch_chk].active_power[r_chk] < 20) {
                          fail_count[ch_chk][r_chk]++;
                          PROCESS_UART_DEBUG_PRINT("  [Round ");
                          PROCESS_UART_DEBUG_PRINT(meas + 1);
                          PROCESS_UART_DEBUG_PRINT("] FAIL: CURRENT OR POWER FAIL | Ch: ");
                          PROCESS_UART_DEBUG_PRINT(ch_chk + 1);
                          PROCESS_UART_DEBUG_PRINT(" Rel: ");
                          PROCESS_UART_DEBUG_PRINT(r_chk + 1);
                          PROCESS_UART_DEBUG_PRINT(" Val: ");
                          PROCESS_UART_DEBUG_PRINTLN(channel_measurements[ch_chk].current[r_chk]);
                      }
                  } else {
                      // This relay was NOT turned on. It MUST measure near zero.
                      if (channel_measurements[ch_chk].current[r_chk] > 120 ||
                          channel_measurements[ch_chk].active_power[r_chk] > 30) {
                          fail_count[ch_chk][r_chk]++;
                          PROCESS_UART_DEBUG_PRINT("  [Round ");
                          PROCESS_UART_DEBUG_PRINT(meas + 1);
                          PROCESS_UART_DEBUG_PRINT("] FAIL: LEAKAGE DETECTED | Ch: ");
                          PROCESS_UART_DEBUG_PRINT(ch_chk + 1);
                          PROCESS_UART_DEBUG_PRINT(" Rel: ");
                          PROCESS_UART_DEBUG_PRINT(r_chk + 1);
                          PROCESS_UART_DEBUG_PRINT(" Val: ");
                          PROCESS_UART_DEBUG_PRINTLN(channel_measurements[ch_chk].current[r_chk]);
                      }
                  }
              }
          }

          // Delay giữa các lần đo (trừ lần cuối)
          if (meas < NUM_RELAY_MEASUREMENTS - 1) {
              delay(650);
          }
      } // end meas loop

      // 2.5 Log tóm tắt pass/fail count cho relay index j
      PROCESS_UART_DEBUG_PRINTLN("  --- Measurement Summary ---");
      for (int ch_chk = 0; ch_chk < 4; ch_chk++) {
          int pass_count = NUM_RELAY_MEASUREMENTS - fail_count[ch_chk][j];
          PROCESS_UART_DEBUG_PRINT("  Ch: ");
          PROCESS_UART_DEBUG_PRINT(ch_chk + 1);
          PROCESS_UART_DEBUG_PRINT(" Rel: ");
          PROCESS_UART_DEBUG_PRINT(j + 1);
          PROCESS_UART_DEBUG_PRINT(" | Pass: ");
          PROCESS_UART_DEBUG_PRINT(pass_count);
          PROCESS_UART_DEBUG_PRINT("/");
          PROCESS_UART_DEBUG_PRINT(NUM_RELAY_MEASUREMENTS);
          PROCESS_UART_DEBUG_PRINT(" | Fail: ");
          PROCESS_UART_DEBUG_PRINT(fail_count[ch_chk][j]);
          PROCESS_UART_DEBUG_PRINT("/");
          PROCESS_UART_DEBUG_PRINTLN(NUM_RELAY_MEASUREMENTS);
      }

      // 3. Verify: Chỉ mark FAIL nếu TẤT CẢ lần đo đều fail
      for (int ch_chk = 0; ch_chk < 4; ch_chk++) {
          for (int r_chk = 0; r_chk < 3; r_chk++) {
              if (fail_count[ch_chk][r_chk] == NUM_RELAY_MEASUREMENTS) {
                  check_relay_result[ch_chk][r_chk] = false; // Mark Fail (cả 3 lần fail)
                  bool should_be_on = (r_chk == j);
                  if (should_be_on) {
                      PROCESS_UART_DEBUG_PRINT("CONFIRMED FAIL: CURRENT OR POWER FAIL (3/3) | Ch: ");
                  } else {
                      PROCESS_UART_DEBUG_PRINT("CONFIRMED FAIL: LEAKAGE DETECTED (3/3) | Ch: ");
                  }
                  PROCESS_UART_DEBUG_PRINT(ch_chk + 1);
                  PROCESS_UART_DEBUG_PRINT(" Rel: ");
                  PROCESS_UART_DEBUG_PRINTLN(r_chk + 1);
              }
          }
      }

      // 4. Turn OFF Relay 'j' for all channels
      for (int i = 0; i < 4; i++) {
           relayService.setRelayState(i * 3 + j, false);
      }
  }

  //==================STATE 6: GET RESULT (FINAL RELAY STATE)==================//
  // Kiểm tra kết quả và điều khiển relay
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      bool system_checks_ok = (LOW_THRESHOLD<=kVoltage[i]<=HIGH_THRESHOLD) &
                              (LOW_THRESHOLD<=kCurrent[i][j]<=HIGH_THRESHOLD) &
                              (LOW_THRESHOLD<=kPower[i][j]<=HIGH_THRESHOLD) &
                              write_eeprom_success[i] &
                              zero_detect_get_result(i);
      
      // If ANY check failed (Leakage OR System), force OFF. Else ON if configured to stay ON.
      if (!check_relay_result[i][j] || !system_checks_ok) {
          relayService.setRelayState(i * 3 + j, false);
      }
      else {
        relayService.setRelayState(i * 3 + j, true);
      }
    }
  }

  // Output log results
  for (int i = 0; i < 4; i++) {
    bool zero_ok = zero_detect_get_result(i);
    if (!zero_ok) {
      PROCESS_UART_DEBUG_PRINT("Zero detect is fail for channel: ");
      PROCESS_UART_DEBUG_PRINTLN(i + 1);
    } else {
      PROCESS_UART_DEBUG_PRINT("Zero detect is pass for channel: ");
      PROCESS_UART_DEBUG_PRINTLN(i + 1);
    }
  }

  // Write eeprom status
  for (int i = 0; i < 4; i++) {
    PROCESS_UART_DEBUG_PRINT("Write eeprom is:");
    PROCESS_UART_DEBUG_PRINT(write_eeprom_success[i] ? "Success" : "Fail");
    PROCESS_UART_DEBUG_PRINT(" for channel: ");
    PROCESS_UART_DEBUG_PRINTLN(i + 1);
  }

  // Output Relay Failure Logs
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 3; j++) {
      if(check_relay_result[i][j] == false) {
        PROCESS_UART_DEBUG_PRINT("Channel ");
        PROCESS_UART_DEBUG_PRINT(i + 1);
        PROCESS_UART_DEBUG_PRINT(" Relay ");
        PROCESS_UART_DEBUG_PRINT(j + 1);
        PROCESS_UART_DEBUG_PRINTLN(" Chap chan relay hoac do sai nang luong");
      } 
    }
  }
 
  PROCESS_UART_DEBUG_PRINTLN("Process done"); 
  PROCESS_UART_DEBUG_PRINT("Time process: ");
  PROCESS_UART_DEBUG_PRINTLN(millis() - start_time_ms);

  //==================STATE 6: END PROCESS - TURN OFF ALL RELAY==================//
  // Warning: If we run this, the LEDs/Relays showing the result will turn OFF immediately.
  // The original user code had this. If you want to SEE the result (Pass=ON), verify this line.
}  

