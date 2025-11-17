#include <Arduino.h>
#include <HardwareSerial.h>
#include "zero_detect.h"

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
    
    Serial5.println("Zero detect init");
    Serial5.print("PC0 pin number: ");
    Serial5.println(ZERO_DETECT_PORT_1);
    Serial5.print("PC1 pin number: ");
    Serial5.println(ZERO_DETECT_PORT_2);
    Serial5.print("PC2 pin number: ");
    Serial5.println(ZERO_DETECT_PORT_3);
    Serial5.print("PC3 pin number: ");
    Serial5.println(ZERO_DETECT_PORT_4);
    
    // Set pin mode trước khi attach interrupt
    pinMode(ZERO_DETECT_PORT_1, INPUT);
    pinMode(ZERO_DETECT_PORT_2, INPUT);
    pinMode(ZERO_DETECT_PORT_3, INPUT);
    pinMode(ZERO_DETECT_PORT_4, INPUT);
    
    // Set up ngắt (STM32 dùng digitalPinToInterrupt)
    // Kiểm tra xem digitalPinToInterrupt có trả về giá trị khác nhau không
    int pin1 = digitalPinToInterrupt(ZERO_DETECT_PORT_1);
    int pin2 = digitalPinToInterrupt(ZERO_DETECT_PORT_2);
    int pin3 = digitalPinToInterrupt(ZERO_DETECT_PORT_3);
    int pin4 = digitalPinToInterrupt(ZERO_DETECT_PORT_4);
    
    Serial5.print("Interrupt pin 1 (PC0): ");
    Serial5.println(pin1);
    Serial5.print("Interrupt pin 2 (PC1): ");
    Serial5.println(pin2);
    Serial5.print("Interrupt pin 3 (PC2): ");
    Serial5.println(pin3);
    Serial5.print("Interrupt pin 4 (PC3): ");
    Serial5.println(pin4);
    
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
    
    // Đo trong 1 giây
    uint32_t time_start = micros();
    while(micros() - time_start < 2000000) {
        delayMicroseconds(10);
    }
    //Stop interrupt
    detachInterrupt(digitalPinToInterrupt(ZERO_DETECT_PORT_1));
    detachInterrupt(digitalPinToInterrupt(ZERO_DETECT_PORT_2));
    detachInterrupt(digitalPinToInterrupt(ZERO_DETECT_PORT_3));
    detachInterrupt(digitalPinToInterrupt(ZERO_DETECT_PORT_4));
    
    // In kết quả
    for (int i = 0; i < 4; i++) {
        Serial5.print("Zero detect count ");
        Serial5.print(i + 1);
        Serial5.print(": ");
        Serial5.println(zero_detect_count[i]);
    }
}

void zero_detect_get_result(uint32_t* counts)
{
    // Trả về kết quả count cho 4 kênh
    if (counts != NULL) {
        for (int i = 0; i < 4; i++) {
            counts[i] = zero_detect_count[i];
        }
    }
}
