// Note các chân sử dụng cho phần mềm I2C
/*
PE13 TIM1_CH3
PE14 TIM1_CH4          
PE15 TIM1_BKIN       
PB13 TIM1_CH1N 
PB14 TIM1_CH2N  
PB15 TIM1_CH3N
PD8 FSMC_D13             
PD9 FSMC_D14 
PD10 FSMC_D15
PD11 FSMC_A16
*/

#include "software_i2c.h"
#include "config.h"

// Biến global để lưu chân hiện tại đang sử dụng
uint8_t SCL_PIN = SCL_PIN_1;
uint8_t SDA_PIN = SDA_PIN_1;

// Hàm delay cho I2C (tần số ~50 kHz với delay 10us)
void i2c_delay() {
    delayMicroseconds(20);
}

// Chọn bus I2C (1-5)
void i2c_select_bus(uint8_t bus_num) {
    switch(bus_num) {
        case 1:
            SCL_PIN = SCL_PIN_1;
            SDA_PIN = SDA_PIN_1;
            break;
        case 2:
            SCL_PIN = SCL_PIN_2;
            SDA_PIN = SDA_PIN_2;
            break;
        case 3:
            SCL_PIN = SCL_PIN_3;
            SDA_PIN = SDA_PIN_3;
            break;
        case 4:
            SCL_PIN = SCL_PIN_4;
            SDA_PIN = SDA_PIN_4;
            break;

        default:
            // Mặc định bus 1 nếu bus_num không hợp lệ
            SCL_PIN = SCL_PIN_1;
            SDA_PIN = SDA_PIN_1;
            break;
    }
}

// Khởi tạo I2C
void i2c_init() {
    pinMode(SCL_PIN, OUTPUT);
    pinMode(SDA_PIN, OUTPUT);
    SCL_HIGH();
    SDA_HIGH();
    i2c_delay();
}

// Tạo điều kiện START
void i2c_start() {
    // Đảm bảo pin SDA ở chế độ OUTPUT
    pinMode(SDA_PIN, OUTPUT);
    SDA_HIGH();
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    SDA_LOW();
    i2c_delay();
    SCL_LOW();
    i2c_delay();
}

// Tạo điều kiện Repeated START (không có STOP trước đó)
void i2c_restart() {
    // Đảm bảo pin SDA ở chế độ OUTPUT
    pinMode(SDA_PIN, OUTPUT);
    SDA_HIGH();
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    SDA_LOW();
    i2c_delay();
    SCL_LOW();
    i2c_delay();
}

// Tạo điều kiện STOP
void i2c_stop() {
    // Đảm bảo pin SDA ở chế độ OUTPUT
    pinMode(SDA_PIN, OUTPUT);
    SDA_LOW();
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    SDA_HIGH();
    i2c_delay();
}

// Ghi 1 byte và đọc ACK
bool i2c_write_byte(uint8_t data) {
    // Đảm bảo pin SDA ở chế độ OUTPUT để ghi dữ liệu
    pinMode(SDA_PIN, OUTPUT);
    
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) {
            SDA_HIGH();
        } else {
            SDA_LOW();
        }
        i2c_delay();
        SCL_HIGH();
        i2c_delay();
        SCL_LOW();
        i2c_delay();
        data <<= 1;
    }
    
    // Release SDA để đọc ACK
    pinMode(SDA_PIN, INPUT_PULLUP);  // INPUT_PULLUP đã tự động kéo pin lên HIGH
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    bool ack = (digitalRead(SDA_PIN) == LOW); // ACK = LOW
    SCL_LOW();
    i2c_delay();
    pinMode(SDA_PIN, OUTPUT);
    return ack;
}



// Đọc 1 byte và gửi ACK/NACK
uint8_t i2c_read_byte(bool ack) {
    uint8_t data = 0;
    pinMode(SDA_PIN, INPUT_PULLUP);
    
    for (uint8_t i = 0; i < 8; i++) {
        SCL_HIGH();
        i2c_delay();
        data = (data << 1) | digitalRead(SDA_PIN);
        SCL_LOW();
        i2c_delay();
    }
    
    // Gửi ACK/NACK
    pinMode(SDA_PIN, OUTPUT);
    if (ack) {
        SDA_LOW();  // ACK
    } else {
        SDA_HIGH(); // NACK
    }
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    SCL_LOW();
    i2c_delay();
    
    return data;
}