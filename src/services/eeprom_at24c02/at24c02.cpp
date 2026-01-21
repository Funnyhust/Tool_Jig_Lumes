#include "at24c02.h"
#include "../software_i2c/software_i2c.h"
#include "../../config.h"

void at24c02_init(uint8_t bus_num) {
    i2c_select_bus(bus_num);
    i2c_init();
}

bool at24c02_write(uint8_t address, uint8_t data) {
    i2c_start();                                    // START condition
    if (!i2c_write_byte(AT24C02_ADDRESS << 1)) {    // Device address + R/W=0 (write)
        i2c_stop();
        return false;
    }
    i2c_write_byte(address);                       // Word address
    i2c_write_byte(data);                          // Data byte
    i2c_stop();                                     // STOP condition
    return true;
}

bool at24c02_write_block(uint8_t address, uint8_t* data, uint8_t length) {
    i2c_start();                                    // START condition
    if (!i2c_write_byte(AT24C02_ADDRESS << 1)) {    // Device address + R/W=0 (write)
        i2c_stop();
        return false;
    }
    i2c_write_byte(address);                       // Word address (n)
    
    // Ghi các byte dữ liệu liên tiếp (tối đa 8 byte cho 1 page)
    for (int i = 0; i < length; i++) {
        i2c_write_byte(data[i]);                   // Data (n+i)
    }
    
    i2c_stop();                                     // STOP condition
    return true;
}

uint8_t at24c02_read(uint8_t address) {
    uint8_t data;
    
    // Gửi word address (write mode)
    i2c_start();
    if (!i2c_write_byte(AT24C02_ADDRESS << 1)) {  // Device address + R/W=0 (write)
        i2c_stop();
        return 0xFF;
    }
    i2c_write_byte(address);                // Word address
    
    // Repeated START và chuyển sang read mode
    i2c_restart();
    if (!i2c_write_byte((AT24C02_ADDRESS << 1) | 0x01)) {  // Device address + R/W=1 (read)
        i2c_stop();
        return 0xFF;
    }
    
    // Đọc 1 byte và gửi NACK
    data = i2c_read_byte(false);  // NACK vì chỉ đọc 1 byte
    
    i2c_stop();
    return data;
}

bool at24c02_read_block(uint8_t address, uint8_t* data, uint8_t length) {
    // Gửi word address (write mode)
    i2c_start();
    if (!i2c_write_byte(AT24C02_ADDRESS << 1)) {  // Device address + R/W=0 (write)
        i2c_stop();
        return false;
    }
    i2c_write_byte(address);                // Word address
    
    // Repeated START và chuyển sang read mode
    i2c_restart();
    if (!i2c_write_byte((AT24C02_ADDRESS << 1) | 0x01)) {  // Device address + R/W=1 (read)
        i2c_stop();
        return false;
    }
    
    // Đọc các byte liên tiếp
    for (uint8_t i = 0; i < length; i++) {
        if (i == length - 1) {
            // Byte cuối cùng: gửi NACK
            data[i] = i2c_read_byte(false);
        } else {
            // Các byte trước: gửi ACK
            data[i] = i2c_read_byte(true);
        }
    }
    
    i2c_stop();
    return true;
}

void at24c02_test(uint8_t bus_num) {
    //read channel 0
    at24c02_init(bus_num);
    uint8_t block_to_read[14];
    at24c02_read_block(0x00, block_to_read, 14);
    //print block_to_read
    for(int i = 0; i < 14; i++){
        UART_DEBUG.print(block_to_read[i]);
        UART_DEBUG.print(" ");
    }
    UART_DEBUG.println();
}