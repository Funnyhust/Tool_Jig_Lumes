#include "process.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdio.h>
#include "../services/bl0906/bl0906.h"
#include "../services/uart/uart_service.h"
#include "../services/relay/relay_service.h"
#include "../services/zero_detect/zero_detect.h"
#include "../config.h"
// Khai báo hàm delay có blink LED từ main.cpp
extern void delayWithBlink(uint32_t ms);


// 4 uart bl0906, 1 uart debug
// Khởi tạo trong hàm để tránh lỗi undefined reference
static UartService* uartBl0906[4] = {NULL, NULL, NULL, NULL};
static UartService* uartDebug = NULL;

// Lưu giá trị đo được cho từng kênh BL0906
static measurement_value_t channel_measurements[4];

// Kết quả zero detect cho 4 kênh
bool zero_detect_result[4] = {true, true, true, true};
static void is_channel_pass(int channel)
{
    if (channel < 0 || channel >= 4) {
        return;
    }
    
    // Kiểm tra voltage có trong ngưỡng cho phép không
    float voltage = channel_measurements[channel].voltage;
    channel_measurements[channel].voltage_ok = (voltage >= VOLTAGE_THRESHOLD_LOW && voltage <= VOLTAGE_THRESHOLD_HIGH);

    // Kiểm tra current có trong ngưỡng cho phép không (đơn vị: mA)
    float current_1 = channel_measurements[channel].current[0];
    float current_2 = channel_measurements[channel].current[1];
    float current_3 = channel_measurements[channel].current[2];


    channel_measurements[channel].current_1_ok = (current_1 >= CURRENT_THRESHOLD_LOW && current_1 <= CURRENT_THRESHOLD_HIGH);
    channel_measurements[channel].current_2_ok = (current_2 >= CURRENT_THRESHOLD_LOW && current_2 <= CURRENT_THRESHOLD_HIGH);
    channel_measurements[channel].current_3_ok = (current_3 >= CURRENT_THRESHOLD_LOW && current_3 <= CURRENT_THRESHOLD_HIGH);
    
    // Kiểm tra power có trong ngưỡng cho phép không (đơn vị: W)
    float power_1 = channel_measurements[channel].active_power[0];
    float power_2 = channel_measurements[channel].active_power[1];
    float power_3 = channel_measurements[channel].active_power[2];


    channel_measurements[channel].power_1_ok = (power_1 >= POWER_THRESHOLD_LOW && power_1 <= POWER_THRESHOLD_HIGH);
    channel_measurements[channel].power_2_ok = (power_2 >= POWER_THRESHOLD_LOW && power_2 <= POWER_THRESHOLD_HIGH);
    channel_measurements[channel].power_3_ok = (power_3 >= POWER_THRESHOLD_LOW && power_3 <= POWER_THRESHOLD_HIGH);
}




// Khởi tạo bl0906
void process_init(void)
{
    // STM32F103VE có 5 UART: Serial1, Serial2, Serial3, Serial4, Serial5
    // Serial1 → Debug
    // Serial2, Serial3, Serial4, Serial5 → 4 kênh BL0906 (channel 0, 1, 2, 3)
    
    // Serial1 cho debug
    if (true) {  // Serial1 luôn có trên STM32F103VE
        UART_DEBUG.begin(19200);
        uartDebug = new UartService(&UART_DEBUG, "DEBUG");
    }
    
    // Serial2 cho BL0906 kênh 0
    if (true) {  // Serial2 luôn có trên STM32F103VE
        UART_BL0906_1.begin(19200);
        uartBl0906[0] = new UartService(&UART_BL0906_1, "BL0906_1");
    }
    
    // Serial3 cho BL0906 kênh 1
    #ifdef USART3
    UART_BL0906_2.begin(19200);
    uartBl0906[1] = new UartService(&UART_BL0906_2, "BL0906_2");
    #endif
    
    // Serial4 cho BL0906 kênh 2
    #if defined(UART4) || defined(USART4)
    UART_BL0906_3.begin(19200);
    uartBl0906[2] = new UartService(&UART_BL0906_3, "BL0906_3");
    #endif
    
    // Serial5 cho BL0906 kênh 3
    #if defined(UART5) || defined(USART5)
    UART_BL0906_4.begin(19200);
    uartBl0906[3] = new UartService(&UART_BL0906_4, "BL0906_4");
    #endif
    
    // Khởi tạo bl0906 cho các UART đã khởi tạo
    for (int i = 0; i < 4; i++) {
        if (uartBl0906[i] != NULL) {
            bl0906_init(NULL, uartBl0906[i]);
        }
    }

    
    // Set gain cho từng kênh (phải set lại UART trước mỗi lần vì p_uart_service bị ghi đè)
    for (int i = 0; i < 4; i++) {
        if (uartBl0906[i] != NULL) {
            // QUAN TRỌNG: Set lại UART và channel trước khi gọi bl0906_proc()
            // vì p_uart_service bị ghi đè khi init các kênh sau
            bl0906_set_uart(uartBl0906[i]);
            bl0906_set_channel(i);
            bl0906_proc();  // Kiểm tra và set gain cho kênh này
            delayWithBlink(100);
        }
    }
}
// Khai báo extern relayService từ main.cpp
extern RelayService relayService;

// Helper function: Đọc từ một BL0906 cụ thể
static void read_bl0906_channel(int channel, UartService* uart)
{
    uint32_t start_time_ms = millis();
    if (uart == NULL) {
        return;
    }
    
    // Set UART và channel cho bl0906
    bl0906_set_uart(uart);
    // Đảm bảo UART được set đúng (quan trọng cho kênh 0)
    if (uart == NULL || uart->getSerial() == NULL) {
        UART_DEBUG.print("ERROR: Channel ");
        UART_DEBUG.print(channel);
        UART_DEBUG.println(" UART is NULL!");
        return;
    }
    // Flush UART để đảm bảo buffer sạch trước khi set gain (quan trọng cho kênh 0)
    uart->getSerial()->flush();
    bl0906_set_channel(channel);  // Set channel trước khi đọc (sẽ tự động reset giá trị)
  
    
    // Kiểm tra và set gain cho kênh này (đảm bảo gain đúng trước khi đọc)
    bl0906_proc();
    // Delay sau khi set gain để BL0906 có thời gian cập nhật giá trị
    // Khi set gain xong, BL0906 cần thời gian để áp dụng gain mới và cập nhật các giá trị đo được
    delayWithBlink(5);  // Tăng delay để đảm bảo gain được áp dụng xong
    
    // Đọc giá trị
    bl0906_send_get_current();

    bl0906_get_voltage();

    bl0906_get_active_power();
   
    // Lưu giá trị vào mảng của kênh này ngay sau khi đọc
    channel_measurements[channel] = bl0906_get_all_measurements();
    
    //Logging giá trị đo được bằng serial4 (chia nhỏ để không block LED blink)
    UART_DEBUG.print("Channel: ");
    UART_DEBUG.println(channel);
    UART_DEBUG.print("Voltage: ");
    UART_DEBUG.println(channel_measurements[channel].voltage);
    UART_DEBUG.print("Current[0]: ");  
    UART_DEBUG.println(channel_measurements[channel].current[0]);
    UART_DEBUG.print("Current[1]: ");
    UART_DEBUG.println(channel_measurements[channel].current[1]);
    UART_DEBUG.print("Current[2]: ");
    UART_DEBUG.println(channel_measurements[channel].current[2]);
    UART_DEBUG.print("Active Power[0]: ");
    UART_DEBUG.println(channel_measurements[channel].active_power[0]);
    UART_DEBUG.print("Active Power[1]: ");
    UART_DEBUG.println(channel_measurements[channel].active_power[1]);
    UART_DEBUG.print("Active Power[2]: ");
    UART_DEBUG.println(channel_measurements[channel].active_power[2]);
    UART_DEBUG.println("--------------------------------");
}

// Khai báo biến điều khiển LED từ main.cpp
extern volatile bool ledBlinkEnable;

void start_process(void)
{
  // turn on all relay
   uint32_t start_time_ms = millis();

   delayWithBlink(10);
   // Bật blink LED
   ledBlinkEnable = true;
   
   // Set UART và channel cho từng kênh (có delay đặc biệt cho kênh 0)
   for (int i = 0; i < 4; i++) {
       if (uartBl0906[i] != NULL) {
           bl0906_set_uart(uartBl0906[i]);
           delay(5);  // Đợi UART được set xong
           bl0906_set_channel(i);
           // Delay lâu hơn cho kênh 0 để đảm bảo channel được set đúng
       }
   }

   // Đọc từ tất cả 4 kênh BL0906
   for (int i = 0; i < 4; i++) {
       if (uartBl0906[i] != NULL) {
           read_bl0906_channel(i, uartBl0906[i]);
       }
       delayWithBlink(2);
   }
   // Đợi 3 giây và đo zero detect trong lúc này
   // Reset zero detect count trước khi đo
   // (zero_detect_process sẽ tự reset và đo trong 2 giây)
   zero_detect_process();
   
   // Kiểm tra kết quả và điều khiển relay
   for (int i = 0; i < 4; i++) {
       is_channel_pass(i);
       
       // Debug: In giá trị để kiểm tra
       /*
              UART_DEBUG.print("Channel ");
       UART_DEBUG.print(i);
       UART_DEBUG.print(" - Zero detect: ");
   
       UART_DEBUG.print(zero_ok ? "PASS" : "FAIL");
       UART_DEBUG.print(", Current_1_ok: ");

       UART_DEBUG.print(channel_measurements[i].current_1_ok);
       UART_DEBUG.print(", Current_2_ok: ");
       UART_DEBUG.print(channel_measurements[i].current_2_ok);
       UART_DEBUG.print(", Current_3_ok: ");
       UART_DEBUG.println(channel_measurements[i].current_3_ok);
       
       
       
       */

       // Kiểm tra zero detect: nếu không pass thì tắt tất cả relay của kênh đó
       bool zero_ok = zero_detect_get_result(i);
       if(!zero_ok) {
           // Zero detect fail - tắt cả 3 relay của kênh này
           UART_DEBUG.print("Zero detect fail for channel: ");
           UART_DEBUG.println(i);
           zero_detect_result[i] = false;
           relayService.setRelayState(i*3, false);
           relayService.setRelayState(i*3+1, false);
           relayService.setRelayState(i*3+2, false);
       } else {
           // Zero detect pass - điều khiển relay theo kết quả current
           zero_detect_result[i] = true;
           UART_DEBUG.print("Channel ");
           UART_DEBUG.print(i);
           UART_DEBUG.print(" - Setting relays: ");
           UART_DEBUG.print(channel_measurements[i].current_1_ok);
           UART_DEBUG.print(", ");
           UART_DEBUG.print(channel_measurements[i].current_2_ok);
           UART_DEBUG.print(", ");
           UART_DEBUG.println(channel_measurements[i].current_3_ok);
           relayService.setRelayState(i*3, channel_measurements[i].current_1_ok&channel_measurements[i].power_1_ok);
           relayService.setRelayState(i*3+1, channel_measurements[i].current_2_ok&channel_measurements[i].power_2_ok);
           relayService.setRelayState(i*3+2, channel_measurements[i].current_3_ok&channel_measurements[i].power_3_ok);
       }
   }

    UART_DEBUG.print("Time process: ");
    UART_DEBUG.println(millis() - start_time_ms);

}

