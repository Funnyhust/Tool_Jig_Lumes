#include "process.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdio.h>
#include "../services/bl0906/bl0906.h"
#include "../services/uart/uart_service.h"
#include "../services/relay/relay_service.h"

// Khai báo hàm delay có blink LED từ main.cpp
extern void delayWithBlink(uint32_t ms);

// 4 uart bl0906, 1 uart debug
// Khởi tạo trong hàm để tránh lỗi undefined reference
static UartService* uartBl0906[4] = {NULL, NULL, NULL, NULL};
static UartService* uartDebug = NULL;

// Lưu giá trị đo được cho từng kênh BL0906
static measurement_value_t channel_measurements[4];
static void is_channel_pass(int channel)
{
    if (channel < 0 || channel >= 4) {
        return;
    }
    
    // Kiểm tra voltage có trong ngưỡng cho phép không
    float voltage = channel_measurements[channel].voltage;
    channel_measurements[channel].voltage_ok = (voltage >= VOLTAGE_THRESHOLD_LOW && voltage <= VOLTAGE_THRESHOLD_HIGH);
    
    // Kiểm tra current có trong ngưỡng cho phép không (đơn vị: mA, chuyển sang A để so sánh)
    uint8_t current_1 = channel_measurements[channel].current[0];
    uint8_t current_2 = channel_measurements[channel].current[1];
    uint8_t current_3 = channel_measurements[channel].current[2];
    
    channel_measurements[channel].current_1_ok = (current_1 >= CURRENT_THRESHOLD_LOW && current_1 <= CURRENT_THRESHOLD_HIGH);
    channel_measurements[channel].current_2_ok = (current_2 >= CURRENT_THRESHOLD_LOW && current_2 <= CURRENT_THRESHOLD_HIGH);
    channel_measurements[channel].current_3_ok = (current_3 >= CURRENT_THRESHOLD_LOW && current_3 <= CURRENT_THRESHOLD_HIGH);
}

static bool _voltage_ok = false;
static bool _current_1_ok = false;
static bool _active_power_ok = false;

static uint16_t _voltage_value = 0;
static uint16_t _current_value_1 = 0;
static uint16_t _current_value_2 = 0;
static uint16_t _current_value_3 = 0;
static uint16_t _active_power_value = 0;



// Khởi tạo bl0906
void process_init(void)
{
    // STM32F103VE có 5 UART: Serial1, Serial2, Serial3, Serial4, Serial5
    // Khởi tạo và begin() để đảm bảo Serial được link vào binary
    // Chỉ khởi tạo các Serial có sẵn (kiểm tra bằng cách gọi begin())
    if (true) {  // Serial1 luôn có trên STM32F103VE
        Serial1.begin(19200);
        uartBl0906[0] = new UartService(&Serial1, "BL0906_1");
    }
    if (true) {  // Serial2 luôn có trên STM32F103VE
        Serial2.begin(19200);
        uartBl0906[1] = new UartService(&Serial2, "BL0906_2");
    }
    // Serial3, Serial4, Serial5 có thể không có trên một số variant
    // Thử khởi tạo và bắt lỗi nếu không có
    #ifdef USART3
    Serial3.begin(19200);
    uartBl0906[2] = new UartService(&Serial3, "BL0906_3");
    #endif
    #if defined(UART4) || defined(USART4)
    Serial4.begin(19200);
    uartBl0906[3] = new UartService(&Serial4, "BL0906_4");
    #endif
    #if defined(UART5) || defined(USART5)
    Serial5.begin(115200);
    uartDebug = new UartService(&Serial5, "DEBUG");
    #endif
    
    // Khởi tạo bl0906 cho các UART đã khởi tạo
    for (int i = 0; i < 4; i++) {
        if (uartBl0906[i] != NULL) {
            bl0906_init(NULL, uartBl0906[i]);
        }
    }
}
// Khai báo extern relayService từ main.cpp
extern RelayService relayService;

// Helper function: Đọc từ một BL0906 cụ thể
static void read_bl0906_channel(int channel, UartService* uart)
{
    if (uart == NULL) {
        return;
    }
    
    // Set UART và channel cho bl0906
    bl0906_set_uart(uart);
    bl0906_set_channel(channel);  // Set channel trước khi đọc (sẽ tự động reset giá trị)
    // Đọc giá trị
    bl0906_send_get_current();
    //delayWithBlink(200); // Đợi đọc xong (3 lần đọc current) - blink LED trong lúc đợi
    bl0906_get_voltage();
    //delayWithBlink(50);  // Blink LED trong lúc đợi
    bl0906_get_active_power();
    // Lưu giá trị vào mảng của kênh này ngay sau khi đọc
    channel_measurements[channel] = bl0906_get_all_measurements();
}

// Khai báo biến điều khiển LED từ main.cpp
extern volatile bool ledBlinkEnable;

void start_process(void)
{
  // turn on all relay
   uint32_t start_time_ms = millis();
   relayService.turnOnAll();
   delayWithBlink(200);
   // Bật blink LED
   ledBlinkEnable = true;
   
   for (int i = 0; i < 4; i++) {
       if (uartBl0906[i] != NULL) {
           bl0906_set_uart(uartBl0906[i]);
           bl0906_set_channel(i);
       }
   }
   
   // Đọc từ tất cả 4 kênh BL0906
   for (int i = 0; i < 4; i++) {
       if (uartBl0906[i] != NULL) {
           read_bl0906_channel(i, uartBl0906[i]);
       }
   }
   
   // Đợi 3 giây (LED sẽ blink trong lúc này)
    while (millis() - start_time_ms < 3000) {
         delayWithBlink(100);
    }
   // Tắt blink LED
   ledBlinkEnable = false;
   
   for (int i = 0; i < 4; i++) {
    is_channel_pass(i);
    relayService.setRelayState(i*3, channel_measurements[i].current_1_ok);
    relayService.setRelayState(i*3+1, channel_measurements[i].current_2_ok);
    relayService.setRelayState(i*3+2, channel_measurements[i].current_3_ok);
 }
}

