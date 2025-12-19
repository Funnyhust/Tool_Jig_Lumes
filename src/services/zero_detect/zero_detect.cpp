#include <Arduino.h>
#include <HardwareSerial.h>
#include "zero_detect.h"
#include "../../config.h"
#include "wiring_constants.h"

// Khai báo hàm delayWithBlink từ main.cpp
extern void delayWithBlink(uint32_t ms);

// Biến count phải là volatile vì được cập nhật trong ISR
static volatile uint32_t zero_detect_count[4] = {0, 0, 0, 0};
static bool zero_detect_initialized = false;

// ISR handlers cho 4 kênh - thêm debug để biết interrupt nào được gọi

uint32_t last_interrupt_time_1 = 0;
uint32_t last_interrupt_time_2 = 0;
uint32_t last_interrupt_time_3 = 0;
uint32_t last_interrupt_time_4 = 0;
void zero_detect_interrupt_1(void) {
  uint32_t current_interrupt_time_1 = micros();
  if(current_interrupt_time_1 - last_interrupt_time_1 < 2000){
    return;
  }
  last_interrupt_time_1 = current_interrupt_time_1;
  zero_detect_count[0]++;

}
void zero_detect_interrupt_2(void) { 
    uint32_t current_interrupt_time_2 = micros();
    if(current_interrupt_time_2 - last_interrupt_time_2 < 2000){
        return;
    }
    last_interrupt_time_2 = current_interrupt_time_2;
    zero_detect_count[1]++; 
    // UART_DEBUG.println("ISR2");
}
void zero_detect_interrupt_3(void) { 
    uint32_t current_interrupt_time_3 = micros();
    if(current_interrupt_time_3 - last_interrupt_time_3 < 2000){
        return;
    }
    last_interrupt_time_3 = current_interrupt_time_3;
    zero_detect_count[2]++; 
    // UART_DEBUG.println("ISR3");
}
    void zero_detect_interrupt_4(void) { 
        uint32_t current_interrupt_time_4 = micros();
    if(current_interrupt_time_4 - last_interrupt_time_4 < 2000){
        return;
    }
    last_interrupt_time_4 = current_interrupt_time_4;
    zero_detect_count[3]++; 
    // UART_DEBUG.println("ISR4");
}


void zero_detect_init_pin(void)
{
    pinMode(ZERO_DETECT_PORT_1, INPUT_PULLUP);
    pinMode(ZERO_DETECT_PORT_2, INPUT_PULLUP);
    pinMode(ZERO_DETECT_PORT_3, INPUT_PULLUP);
    pinMode(ZERO_DETECT_PORT_4, INPUT_PULLUP);
}

void zero_detect_init(void)
{
    // Chỉ init 1 lần để tránh attach interrupt nhiều lần
    if (zero_detect_initialized) {
        return;
    }
    
    // UART_DEBUG.println("Zero detect init");
    // UART_DEBUG.print("PC0 pin number: ");
    // UART_DEBUG.println(ZERO_DETECT_PORT_1);
    // UART_DEBUG.print("PC1 pin number: ");
    // UART_DEBUG.println(ZERO_DETECT_PORT_2);
    // UART_DEBUG.print("PC2 pin number: ");
    // UART_DEBUG.println(ZERO_DETECT_PORT_3);
    // UART_DEBUG.print("PC3 pin number: ");
    // UART_DEBUG.println(ZERO_DETECT_PORT_4);
    
    // Set pin mode trước khi attach interrupt

    
    // Set up ngắt (STM32 dùng digitalPinToInterrupt)
    // Kiểm tra xem digitalPinToInterrupt có trả về giá trị khác nhau không
    int pin1 = digitalPinToInterrupt(ZERO_DETECT_PORT_1);
    int pin2 = digitalPinToInterrupt(ZERO_DETECT_PORT_2);
    int pin3 = digitalPinToInterrupt(ZERO_DETECT_PORT_3);
    int pin4 = digitalPinToInterrupt(ZERO_DETECT_PORT_4);
    
    // UART_DEBUG.print("Interrupt pin 1 (PC0): ");
    // UART_DEBUG.println(pin1);
    // UART_DEBUG.print("Interrupt pin 2 (PC1): ");
    // UART_DEBUG.println(pin2);
    // UART_DEBUG.print("Interrupt pin 3 (PC2): ");
    // UART_DEBUG.println(pin3);
    // UART_DEBUG.print("Interrupt pin 4 (PC3): ");
    // UART_DEBUG.println(pin4);
    
    // Trên STM32, nếu các pin có cùng interrupt number, chúng sẽ trigger cùng nhau
    // Cần kiểm tra xem có phải vấn đề này không
    if (pin1 == pin2 || pin1 == pin3 || pin1 == pin4 || 
        pin2 == pin3 || pin2 == pin4 || pin3 == pin4) {
        UART_DEBUG.println("WARNING: Some pins share the same interrupt number!");
    }
    for (int i = 0; i < 4; i++) {
        zero_detect_count[i] = 0;
    }
    // Trên STM32, attachInterrupt có thể cần dùng pin number trực tiếp
    attachInterrupt(ZERO_DETECT_PORT_1, zero_detect_interrupt_1, CHANGE);
    attachInterrupt(ZERO_DETECT_PORT_2, zero_detect_interrupt_2, CHANGE);
    attachInterrupt(ZERO_DETECT_PORT_3, zero_detect_interrupt_3, CHANGE);
    attachInterrupt(ZERO_DETECT_PORT_4, zero_detect_interrupt_4, CHANGE);
    
    //UART_DEBUG.println("Interrupts attached");
    
    zero_detect_initialized = true;
}


void zero_detect_process(void)
{
    
    // Đo trong 2 giây
    zero_detect_init();

    // Reset count trước khi đo

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
        UART_DEBUG.print("Zero detect count ");
        UART_DEBUG.print(i + 1);
        UART_DEBUG.print(": ");
        UART_DEBUG.println(zero_detect_count[i]);
    }
    zero_detect_initialized = false;
}

bool zero_detect_get_result(uint8_t channel)
{
    // Trả về kết quả count cho 4 kênh
    // Với 2 giây đo: 50Hz = 100 zero crossing, cho phép 95-105 (tolerance ±5%)
    if (channel < 0 || channel >= 4) {
        return false;
    }
    return zero_detect_count[channel] > 94 && zero_detect_count[channel] < 106;
}


// void zero_detect_process(void)
// {
//     // Đợi một chút để bỏ qua nhiễu từ quá trình bật relay (nếu có)
//     // Delay TRƯỚC khi init để interrupt chưa được attach → không đếm nhiễu
//     delayWithBlink(100);
    
//     // Đảm bảo interrupt đã được init (sau khi đã ổn định)
//     zero_detect_init();
    
//     // QUAN TRỌNG: Reset count trước mỗi lần đo (disable interrupt để tránh race condition)
//     noInterrupts();
//     for (int i = 0; i < 4; i++) {
//         zero_detect_count[i] = 0;
//     }
//     // Bắt đầu đo ngay sau khi reset (không có delay giữa reset và bắt đầu đo)
//     uint32_t time_start = micros();
//     uint32_t duration_us = 1000000;  // 1 giây
//     interrupts();
    
//     // Đo trong 1 giây (1000000 microseconds)
//     // Với 50Hz: 1 giây = 50 zero crossing (RISING edge)
//     while (true) {
//         uint32_t current_time = micros();
//         uint32_t elapsed;
        
//         // Xử lý overflow của micros()
//         if (current_time >= time_start) {
//             elapsed = current_time - time_start;
//         } else {
//             // micros() đã overflow
//             elapsed = (0xFFFFFFFF - time_start) + current_time + 1;
//         }
        
//         if (elapsed >= duration_us) {
//             noInterrupts();
//             break;
//         }
        
//         delayMicroseconds(10);
//     }
//     // Disable interrupt ngay khi hết thời gian đo để tránh ISR đếm tiếp
//     // Copy count vào biến static để đảm bảo kết quả chính xác và có thể dùng sau này

//     for (int i = 0; i < 4; i++) {
//         zero_detect_result[i] = zero_detect_count[i];
//     }
//     interrupts();  // QUAN TRỌNG: Phải enable interrupt lại, nếu không hệ thống sẽ bị treo!
    
//     // In kết quả (đã được copy khi interrupt bị disable)
//     for (int i = 0; i < 4; i++) {
//         UART_DEBUG.print("Zero detect count ");
//         UART_DEBUG.print(i + 1);
//         UART_DEBUG.print(": ");
//         UART_DEBUG.println(zero_detect_result[i]);
//     }
// }