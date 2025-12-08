#ifndef WRITE_MEMORY_H
#define WRITE_MEMORY_H

#include <Arduino.h>
#include <EEPROM.h>


// LƯU Ý: Module này dùng EEPROM emulated của Arduino (lưu vào Flash của STM32)
// KHÔNG phải EEPROM ngoài AT24C02 (EEPROM ngoài được quản lý bởi at24c02.h/cpp)
// Địa chỉ bắt đầu trong Flash memory (EEPROM emulated) để lưu các giá trị threshold
#define SETTING_FRAME_ID                 0xD3


// Mỗi channel: 4 bytes (voltage) + 12 bytes (3 current * 4) + 12 bytes (3 power * 4) = 28 bytes
// Địa chỉ bắt đầu của mỗi channel trong EEPROM
// Channel 0: 0x00, Channel 1: 0x1C (28), Channel 2: 0x38 (56), Channel 3: 0x54 (84)
#define MEMORY_ADDR_CHANNEL_0    0x00
#define MEMORY_ADDR_CHANNEL_1    0x1C
#define MEMORY_ADDR_CHANNEL_2    0x38
#define MEMORY_ADDR_CHANNEL_3    0x54

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khởi tạo module write_memory
 * @note Cần gọi hàm này trước khi sử dụng các hàm khác
 */

typedef struct {
    uint8_t channel;
    uint8_t sub_channel;
    uint32_t voltage_threshold;
    uint32_t current_threshold[4][3];
    uint32_t power_threshold[4][3];
} write_memory_t;

void write_memory_process(uint8_t byte);

/**
 * @brief Đọc dữ liệu threshold của một channel từ EEPROM
 * @param channel Channel number (0-3)
 * @param voltage_threshold Con trỏ để lưu voltage threshold
 * @param current_threshold Con trỏ để lưu current threshold array (3 phần tử)
 * @param power_threshold Con trỏ để lưu power threshold array (3 phần tử)
 * @return true nếu đọc thành công, false nếu thất bại
 */
bool read_eeprom_channel(uint8_t channel, 
                                uint32_t* voltage_threshold,
                                uint32_t* current_threshold,
                                uint32_t* power_threshold);

/**
 * @brief Đọc tất cả các giá trị threshold từ EEPROM cho tất cả 4 channels
 * @param voltage_threshold Mảng 4 phần tử để lưu voltage threshold (mV)
 * @param current_threshold Mảng 4x3 để lưu current threshold (mA)
 * @param power_threshold Mảng 4x3 để lưu power threshold (mW)
 * @return true nếu đọc thành công ít nhất 1 channel, false nếu không
 */
bool read_all_eeprom_channels(uint32_t voltage_threshold[4],
                                     uint32_t current_threshold[4][3],
                                     uint32_t power_threshold[4][3]);

void reset_write_memory_channel(uint8_t channel);

void first_write_memory_all_channels(void);

void read_and_print_all_channels(void);

#ifdef __cplusplus
}
#endif

#endif // WRITE_MEMORY_H

