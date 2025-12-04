#ifndef SOFTWARE_I2C_H
#define SOFTWARE_I2C_H

#include <Arduino.h>

// Định nghĩa các cặp chân I2C
#define SCL_PIN_1 PE13
#define SDA_PIN_1 PE14
#define SCL_PIN_2 PE15
#define SDA_PIN_2 PB13
#define SCL_PIN_3 PB14
#define SDA_PIN_3 PB15
#define SCL_PIN_4 PD8
#define SDA_PIN_4 PD9
#define SCL_PIN_5 PD10
#define SDA_PIN_5 PD11

// Biến global để lưu chân hiện tại đang sử dụng
extern uint8_t SCL_PIN;
extern uint8_t SDA_PIN;

// Thao tác chân cơ bản
#define SDA_HIGH() digitalWrite(SDA_PIN, HIGH)
#define SDA_LOW()  digitalWrite(SDA_PIN, LOW)
#define SCL_HIGH() digitalWrite(SCL_PIN, HIGH)
#define SCL_LOW()  digitalWrite(SCL_PIN, LOW)

// Hàm chọn bus I2C (1-5)
void i2c_select_bus(uint8_t bus_num);

// Hàm khởi tạo I2C
void i2c_init();

// Các hàm giao tiếp I2C
void i2c_start();
void i2c_restart();  // Repeated START condition
void i2c_stop();
bool i2c_write_byte(uint8_t data);
uint8_t i2c_read_byte(bool ack);

#endif // SOFTWARE_I2C_H