#include <Arduino.h>
#include <HardwareSerial.h>
#include "zero_detect.h"

// Khai báo hàm delayWithBlink từ main.cpp
extern void delayWithBlink(uint32_t ms);

// Biến count phải là volatile vì được cập nhật trong ISR
static volatile uint32_t zero_detect_count[4] = {0, 0, 0, 0};
static bool zero_detect_initialized = false;

// ISR handlers cho 4 kênh - thêm debug để biết interrupt nào được gọi
void zero_detect_interrupt_1(void) { 
    zero_detect_count[0]++; 
    // Debug: in ra Serial5 (cẩn thận, Serial5 trong ISR có thể gây vấn đề)
    // Serial5.println("ISR1");
}
void zero_detect_interrupt_2(void) { 
    zero_detect_count[1]++; 
    // Serial5.println("ISR2");
}
void zero_detect_interrupt_3(void) { 
    zero_detect_count[2]++; 
    // Serial5.println("ISR3");
}
void zero_detect_interrupt_4(void) { 
    zero_detect_count[3]++; 
    // Serial5.println("ISR4");
}




void zero_detect_init(void)
{
    // Chỉ init 1 lần để tránh attach interrupt nhiều lần
    if (zero_detect_initialized) {
        return;
    }
    
    Serial.println("Zero detect init");
    Serial.print("PC0 pin number: ");
    Serial.println(ZERO_DETECT_PORT_1);
    Serial.print("PC1 pin number: ");
    Serial.println(ZERO_DETECT_PORT_2);
    Serial.print("PC2 pin number: ");
    Serial.println(ZERO_DETECT_PORT_3);
    Serial.print("PC3 pin number: ");
    Serial.println(ZERO_DETECT_PORT_4);
    
    // Set pin mode trước khi attach interrupt
    pinMode(ZERO_DETECT_PORT_1, INPUT_PULLUP);
    pinMode(ZERO_DETECT_PORT_2, INPUT_PULLUP);
    pinMode(ZERO_DETECT_PORT_3, INPUT_PULLUP);
    pinMode(ZERO_DETECT_PORT_4, INPUT_PULLUP);
    
    // Set up ngắt (STM32 dùng digitalPinToInterrupt)
    // Kiểm tra xem digitalPinToInterrupt có trả về giá trị khác nhau không
    int pin1 = digitalPinToInterrupt(ZERO_DETECT_PORT_1);
    int pin2 = digitalPinToInterrupt(ZERO_DETECT_PORT_2);
    int pin3 = digitalPinToInterrupt(ZERO_DETECT_PORT_3);
    int pin4 = digitalPinToInterrupt(ZERO_DETECT_PORT_4);
    
    Serial.print("Interrupt pin 1 (PC0): ");
    Serial.println(pin1);
    Serial.print("Interrupt pin 2 (PC1): ");
    Serial.println(pin2);
    Serial.print("Interrupt pin 3 (PC2): ");
    Serial.println(pin3);
    Serial.print("Interrupt pin 4 (PC3): ");
    Serial.println(pin4);
    
    // Trên STM32, nếu các pin có cùng interrupt number, chúng sẽ trigger cùng nhau
    // Cần kiểm tra xem có phải vấn đề này không
    if (pin1 == pin2 || pin1 == pin3 || pin1 == pin4 || 
        pin2 == pin3 || pin2 == pin4 || pin3 == pin4) {
        Serial5.println("WARNING: Some pins share the same interrupt number!");
    }
    
    attachInterrupt(pin1, zero_detect_interrupt_1, RISING);
    attachInterrupt(pin2, zero_detect_interrupt_2, RISING);
    attachInterrupt(pin3, zero_detect_interrupt_3, RISING);
    attachInterrupt(pin4, zero_detect_interrupt_4, RISING);
    
    Serial5.println("Interrupts attached");
    
    zero_detect_initialized = true;
}


void zero_detect_process(void)
{
    // Reset count trước khi đo
    for (int i = 0; i < 4; i++) {
        zero_detect_count[i] = 0;
    }
    
    // Đo trong 2 giây
    zero_detect_init();
    uint32_t time_start = micros();
    while(micros() - time_start < 1000000) {
        delayWithBlink(1);
    }
    //Stop interrupt
    detachInterrupt(digitalPinToInterrupt(ZERO_DETECT_PORT_1));
    detachInterrupt(digitalPinToInterrupt(ZERO_DETECT_PORT_2));
    detachInterrupt(digitalPinToInterrupt(ZERO_DETECT_PORT_3));
    detachInterrupt(digitalPinToInterrupt(ZERO_DETECT_PORT_4));
    
    // In kết quả
    for (int i = 0; i < 4; i++) {
        Serial.print("Zero detect count ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(zero_detect_count[i]);
    }
}

bool zero_detect_get_result(uint8_t channel)
{
    // Trả về kết quả count cho 4 kênh
    // Với 2 giây đo: 50Hz = 100 zero crossing, cho phép 95-105 (tolerance ±5%)
    if (channel < 0 || channel >= 4) {
        return false;
    }
    return zero_detect_count[channel] > 45 && zero_detect_count[channel] < 55;
}
