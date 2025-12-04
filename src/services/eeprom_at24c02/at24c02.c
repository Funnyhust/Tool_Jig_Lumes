#include "at24c02.h"
#include "../software_i2c/software_i2c.h"

void at24c02_init(void) {
  for (int i = 1; i <= 4; i++) {
    i2c_select_bus(i);
    i2c_init();
  }
}

void at24c02_write(uint8_t address, uint8_t data) {
    i2c_start();                                    // START condition
    i2c_write_byte(AT24C02_ADDRESS << 1);          // Device address + R/W=0 (write)
    i2c_write_byte(address);                       // Word address
    i2c_write_byte(data);                          // Data byte
    i2c_stop();                                     // STOP condition
}

void at24c02_write_block(uint8_t address, uint8_t* data, uint8_t length) {
    i2c_start();                                    // START condition
    i2c_write_byte(AT24C02_ADDRESS << 1);          // Device address + R/W=0 (write)
    i2c_write_byte(address);                       // Word address (n)
    
    // Ghi các byte dữ liệu liên tiếp (tối đa 8 byte cho 1 page)
    for (int i = 0; i < length; i++) {
        i2c_write_byte(data[i]);                   // Data (n+i)
    }
    
    i2c_stop();                                     // STOP condition
}

uint8_t at24c02_read(uint8_t address) {
    uint8_t data;
    
    // Gửi word address (write mode)
    i2c_start();
    i2c_write_byte(AT24C02_ADDRESS << 1);  // Device address + R/W=0 (write)
    i2c_write_byte(address);                // Word address
    
    // Repeated START và chuyển sang read mode
    i2c_restart();
    i2c_write_byte((AT24C02_ADDRESS << 1) | 0x01);  // Device address + R/W=1 (read)
    
    // Đọc 1 byte và gửi NACK
    data = i2c_read_byte(false);  // NACK vì chỉ đọc 1 byte
    
    i2c_stop();
    return data;
}

void at24c02_read_block(uint8_t address, uint8_t* data, uint8_t length) {
    // Gửi word address (write mode)
    i2c_start();
    i2c_write_byte(AT24C02_ADDRESS << 1);  // Device address + R/W=0 (write)
    i2c_write_byte(address);                // Word address
    
    // Repeated START và chuyển sang read mode
    i2c_restart();
    i2c_write_byte((AT24C02_ADDRESS << 1) | 0x01);  // Device address + R/W=1 (read)
    
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
}