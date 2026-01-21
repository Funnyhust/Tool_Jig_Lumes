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
 * File name: write_memory.cpp
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
#include "write_memory.h"
#include "../../config.h"

#define RESET_SETTING_BUFFER_TIME 1000

// Frame set memory: {ID(1byte), channel_sub(1byte), voltage_threshold(2bytes),
// current_threshold(4bytes), power_threshold(4bytes), checksum(1byte)} tổng 13
// bytes channel_sub: 4 bit cao = channel (0-3), 4 bit thấp = sub_channel (0-2)

// Định nghĩa các giá trị default (chỉ định nghĩa ở đây, không phải trong
// header)
uint32_t FIRST_WRITE_VOLTAGE_THRESHOLD[4] = {
    220000, 220000, 220000, 220000 // mV
};

uint32_t FIRST_WRITE_CURRENT_THRESHOLD[4][3] = {
    {184400, 186500, 185900}, // kênh 1
    {185400, 184600, 188200}, // kênh 2
    {184700, 184000, 185600}, // kênh 3
    {183700, 186200, 185200}  // kênh 4
}; // mA

uint32_t FIRST_WRITE_POWER_THRESHOLD[4][3] = {
    {40650, 40990, 40920}, // kênh 1
    {40780, 40670, 41410}, // kênh 2
    {40470, 40540, 40890}, // kênh 3
    {40420, 41100, 40710}  // kênh 4 Fake sai số để test case kênh sai quá 10%
}; // mW

// Biến để đánh dấu đã khởi tạo chưa
write_memory_t setting_data;

static bool memory_initialized = false;

// Địa chỉ bắt đầu của mỗi channel trong EEPROM
static const uint16_t MEMORY_ADDR_CHANNEL[4] = {
    MEMORY_ADDR_CHANNEL_0, MEMORY_ADDR_CHANNEL_1, MEMORY_ADDR_CHANNEL_2,
    MEMORY_ADDR_CHANNEL_3};

static uint8_t rx_setting_buffer[11];
static uint8_t rx_setting_buffer_index = 0;
static bool check_setting_buffer_full = false;

static void reset_setting_buffer(void) {
  rx_setting_buffer_index = 0;
  check_setting_buffer_full = false;
}
uint32_t time_rx_setting_buffer = 0;
uint32_t last_time_rx_setting_buffer = 0;
static void add_to_setting_buffer(uint8_t data) {
  time_rx_setting_buffer = millis();
  if (time_rx_setting_buffer - last_time_rx_setting_buffer >
      RESET_SETTING_BUFFER_TIME) {
    reset_setting_buffer();
  }
  rx_setting_buffer[rx_setting_buffer_index] = data;
  rx_setting_buffer_index++;
  if (rx_setting_buffer_index == 11) {
    check_setting_buffer_full = true;
  }
  last_time_rx_setting_buffer = time_rx_setting_buffer;
}

static bool calculate_checksum(void) {
  uint8_t checksum = 0;
  for (int i = 0; i < 10; i++) {
    checksum += rx_setting_buffer[i];
  }
  checksum = ~checksum; // Sửa syntax: phải dùng = thay vì ~=
  UART_DEBUG.print("Checksum: ");
  UART_DEBUG.println(rx_setting_buffer[10]);
  UART_DEBUG.print("Checksum expected: ");
  UART_DEBUG.println(checksum);
  return checksum == rx_setting_buffer[10];
}

static bool check_valid_frame(void) {
  // Kiểm tra frame ID - phải bằng 0xD3
  if (rx_setting_buffer[0] != SETTING_FRAME_ID) {
    UART_DEBUG.print("Invalid frame ID. Expected: 0xD3, Received: 0x");
    UART_DEBUG.println(rx_setting_buffer[0], HEX);
    return false;
  }
  // Kiểm tra checksum
  if (!calculate_checksum()) {
    UART_DEBUG.println("Invalid checksum");
    return false;
  }
  return true;
}

static bool mapping_process() {
  if (!check_valid_frame()) {
    UART_DEBUG.print("Invalid frame");
    return false;
  }
  uint32_t data = 0;
  // Byte 1: 4 bit cao = channel (0-3), 4 bit thấp = sub_channel (0-2)
  setting_data.channel = (rx_setting_buffer[1] >> 4) & 0x0F;
  setting_data.sub_channel = rx_setting_buffer[1] & 0x0F;

  // Kiểm tra hợp lệ
  if (setting_data.channel >= 4 || setting_data.sub_channel >= 3) {
    UART_DEBUG.print("Invalid channel or sub_channel");
    return false;
  }

  // Byte 2-5: current_threshold (4 bytes)
  data = ((uint32_t)rx_setting_buffer[2] << 24) |
         ((uint32_t)rx_setting_buffer[3] << 16) |
         ((uint32_t)rx_setting_buffer[4] << 8) | (uint32_t)rx_setting_buffer[5];
  setting_data
      .current_threshold[setting_data.channel][setting_data.sub_channel] = data;

  // Byte 6-9: power_threshold (4 bytes)
  data = ((uint32_t)rx_setting_buffer[6] << 24) |
         ((uint32_t)rx_setting_buffer[7] << 16) |
         ((uint32_t)rx_setting_buffer[8] << 8) | (uint32_t)rx_setting_buffer[9];
  setting_data.power_threshold[setting_data.channel][setting_data.sub_channel] =
      data;
  return true;
}

static void write_memory_init(void) {
  if (memory_initialized) {
    return;
  }

  // Khởi tạo EEPROM emulated (Arduino dùng Flash của STM32 để mô phỏng EEPROM)
  // Đây KHÔNG phải EEPROM ngoài AT24C02 - đó là module riêng trong
  // at24c02.h/cpp Trên STM32, EEPROM.begin() không nhận tham số size
  EEPROM.begin();
  memory_initialized = true;
}

/**
 * @brief Ghi dữ liệu threshold của một channel vào EEPROM
 * @param channel Channel number (0-3)
 * @note Dùng EEPROM.put() để ghi cả uint32_t một lúc, không cần ghi từng byte
 */
static void write_memory_to_eeprom_channel(uint8_t channel,
                                           uint8_t sub_channel) {
  if (!memory_initialized) {
    write_memory_init();
  }

  if (channel >= 4 || sub_channel >= 3) {
    return; // Invalid channel or sub_channel
  }

  uint16_t base_addr = MEMORY_ADDR_CHANNEL[channel];

  // Ghi current_threshold cho sub_channel này
  // Offset: base_addr + 4 (bỏ qua voltage) + sub_channel * 4
  EEPROM.put(base_addr + 4 + sub_channel * sizeof(uint32_t),
             setting_data.current_threshold[channel][sub_channel]);

  // Ghi power_threshold cho sub_channel này
  // Offset: base_addr + 4 (voltage) + 12 (3 current) + sub_channel * 4 =
  // base_addr + 16 + sub_channel * 4
  EEPROM.put(base_addr + 16 + sub_channel * sizeof(uint32_t),
             setting_data.power_threshold[channel][sub_channel]);

  // Trên STM32, EEPROM tự động commit, không cần gọi EEPROM.commit()
}

// Clear memory for channel i
static void write_memory_clear_channel(uint8_t channel) {
  if (!memory_initialized) {
    write_memory_init();
  }

  // Xóa tất cả dữ liệu của channel (ghi 0xFF)
  uint16_t base_addr = MEMORY_ADDR_CHANNEL[channel];
  for (uint16_t i = 0; i < 28; i++) {
    EEPROM.write(base_addr + i, 0xFF);
  }

  // Trên STM32, EEPROM tự động commit, không cần gọi EEPROM.commit()
  UART_DEBUG.print("Write memory: Cleared channel ");
  UART_DEBUG.println(channel);
}

void write_memory_process(uint8_t byte) {
  add_to_setting_buffer(byte);
  if (check_setting_buffer_full) {
    if (!mapping_process()) {
      reset_setting_buffer();
      return;
    }
    // Ghi dữ liệu vào EEPROM cho channel và sub_channel này
    write_memory_to_eeprom_channel(setting_data.channel,
                                   setting_data.sub_channel);
    reset_setting_buffer();
    UART_DEBUG.print("Write memory success for channel ");
    UART_DEBUG.print(setting_data.channel);
    UART_DEBUG.print(", sub_channel ");
    UART_DEBUG.println(setting_data.sub_channel);
  }
}

/**
 * @brief Đọc dữ liệu threshold của một channel từ EEPROM
 * @param channel Channel number (0-3)
 * @param voltage_threshold Con trỏ để lưu voltage threshold (4 bytes)
 * @param current_threshold Con trỏ để lưu current threshold array (3 * 4 = 12
 * bytes)
 * @param power_threshold Con trỏ để lưu power threshold array (3 * 4 = 12
 * bytes)
 * @return true nếu đọc thành công, false nếu channel không hợp lệ
 */
bool read_eeprom_channel(uint8_t channel, uint32_t *voltage_threshold,
                         uint32_t *current_threshold,
                         uint32_t *power_threshold) {
  if (!memory_initialized) {
    write_memory_init();
  }

  if (channel >= 4 || voltage_threshold == NULL || current_threshold == NULL ||
      power_threshold == NULL) {
    return false; // Invalid channel or NULL pointer
  }

  uint16_t base_addr = MEMORY_ADDR_CHANNEL[channel];

  // Đọc voltage_threshold (4 bytes) - dùng EEPROM.get() để đọc cả uint32_t
  EEPROM.get(base_addr, *voltage_threshold);

  // Đọc current_threshold[3] (3 * 4 = 12 bytes)
  for (uint8_t i = 0; i < 3; i++) {
    EEPROM.get(base_addr + 4 + i * sizeof(uint32_t), current_threshold[i]);
  }

  // Đọc power_threshold[3] (3 * 4 = 12 bytes)
  for (uint8_t i = 0; i < 3; i++) {
    EEPROM.get(base_addr + 4 + 12 + i * sizeof(uint32_t), power_threshold[i]);
  }

  return true;
}

/**
 * @brief Đọc tất cả các giá trị threshold từ EEPROM cho tất cả 4 channels
 * @param voltage_threshold Mảng 4 phần tử để lưu voltage threshold
 * @param current_threshold Mảng 4x3 để lưu current threshold
 * @param power_threshold Mảng 4x3 để lưu power threshold
 * @return true nếu đọc thành công ít nhất 1 channel, false nếu không
 */
bool read_all_eeprom_channels(uint32_t voltage_threshold[4],
                              uint32_t current_threshold[4][3],
                              uint32_t power_threshold[4][3]) {
  bool success = false;

  for (uint8_t ch = 0; ch < 4; ch++) {
    if (read_eeprom_channel(ch, &voltage_threshold[ch], current_threshold[ch],
                            power_threshold[ch])) {
      success = true;
    }
  }

  return success;
}

void first_write_memory_all_channels(void) {
  // Dùng giá trị default để ghi vào EEPROM
  for (uint8_t ch = 0; ch < 4; ch++) {
    setting_data.channel = ch;
    setting_data.voltage_threshold = FIRST_WRITE_VOLTAGE_THRESHOLD[ch];
    // Ghi voltage cho channel này
    uint16_t base_addr = MEMORY_ADDR_CHANNEL[ch];
    EEPROM.put(base_addr, setting_data.voltage_threshold);

    // Ghi từng sub_channel (current và power)
    for (uint8_t sub = 0; sub < 3; sub++) {
      setting_data.sub_channel = sub;
      setting_data.current_threshold[ch][sub] =
          FIRST_WRITE_CURRENT_THRESHOLD[ch][sub];
      setting_data.power_threshold[ch][sub] =
          FIRST_WRITE_POWER_THRESHOLD[ch][sub];
      write_memory_to_eeprom_channel(ch, sub);
    }
  }
}
