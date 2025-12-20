#include <Arduino.h>
#include <HardwareSerial.h>
#include "stm32f1xx_hal.h"
#include "services/relay/relay_service.h"
#include "app/process.h"
#include "services/zero_detect/zero_detect.h"
#include "services/eeprom_at24c02/at24c02.h"
#include "services/write_memory/write_memory.h"
#include "services/control_power/control_power.h"
#include "config.h"
#include <IWatchdog.h>

#define  WATCHDOG_TIMEOUT_MS 2000

RelayService relayService;

// PC13 là LED tích hợp trên STM32F103VE
#define LED_PIN PC13

// Biến để điều khiển blink LED
volatile bool ledBlinkEnable = false;
volatile bool ledState = false;
volatile uint32_t ledLastBlinkTime = 0;

// Hàm delay có blink LED
void delayWithBlink(uint32_t ms) {
    uint32_t startTime = millis();
    while (millis() - startTime < ms) {
        if (ledBlinkEnable) {
            uint32_t currentTime = millis();
            if (currentTime - ledLastBlinkTime >= 200) {  // Blink mỗi 200ms
                ledState = !ledState;
                digitalWrite(LED_PIN, ledState ? HIGH : LOW);
                ledLastBlinkTime = currentTime;
            }
        }
        delay(1);  // Delay nhỏ để không chiếm CPU
    }
}

// Hàm kiểm tra trạng thái clock
void checkClockStatus() {
    UART_DEBUG.println("\n=== Clock Status Check ===");
    UART_DEBUG.print("SystemCoreClock: ");
    UART_DEBUG.println(SystemCoreClock);
    
    // Kiểm tra HSE
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY)) {
        UART_DEBUG.println("HSE: READY");
    } else {
        UART_DEBUG.println("HSE: NOT READY");
    }
    
    // Kiểm tra PLL
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)) {
        UART_DEBUG.println("PLL: READY");
    } else {
        UART_DEBUG.println("PLL: NOT READY");
    }
    
    // Kiểm tra SYSCLK source
    uint32_t sysclk_source = __HAL_RCC_GET_SYSCLK_SOURCE();
    UART_DEBUG.print("SYSCLK Source: ");
    switch(sysclk_source) {
        case RCC_SYSCLKSOURCE_STATUS_HSI:
            UART_DEBUG.println("HSI");
            break;
        case RCC_SYSCLKSOURCE_STATUS_HSE:
            UART_DEBUG.println("HSE (8MHz)");
            break;
        case RCC_SYSCLKSOURCE_STATUS_PLLCLK:
            UART_DEBUG.println("PLL (72MHz)");
            break;
        default:
            UART_DEBUG.print("Unknown (0x");
            UART_DEBUG.print(sysclk_source, HEX);
            UART_DEBUG.println(")");
            break;
    }
    
    // Kiểm tra RCC register
    UART_DEBUG.print("RCC_CFGR: 0x");
    UART_DEBUG.println(RCC->CFGR, HEX);
    UART_DEBUG.print("RCC_CR: 0x");
    UART_DEBUG.println(RCC->CR, HEX);
    UART_DEBUG.println("========================\n");
}
bool is_first_run = true;
uint32_t time_end_process=0;
void setup() {
    // Khởi tạo LED pin
    /*
        // ============= Clock Status Check =============
    // Khởi tạo Serial trước để có thể debug
    UART_DEBUG.begin(19200);
    delay(100);
    
    // Kiểm tra trạng thái clock
    checkClockStatus();
    // ============= Clock Status Check =============
    UART_DEBUG.print("CPU clock: ");
    //UART_DEBUG.println(F_CPU);
    UART_DEBUG.println(SystemCoreClock);
    */
    uart_init();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    control_power_init();

    //IWatchdog.begin(WATCHDOG_TIMEOUT_MS);
}

uint32_t count = 0;
void loop() {
    if(is_return_power_control_signal()) {
        delay(500);
        digitalWrite(LED_PIN, LOW);
        control_power_on();
        relayService.init();
        zero_detect_init_pin();
        delay(1000);
        process_init();
        //first_write_memory_all_channels();
        ledBlinkEnable = true;
        start_process();
        time_end_process = millis();
        ledBlinkEnable = false;
        digitalWrite(LED_PIN, HIGH);

    } else {
        control_power_shutdown();
    }
    if(millis() - time_end_process >= 30000){
        relayService.turnOffAll();
        delay(1000);
    }
    if(UART_DEBUG.available()){
        uint8_t data = UART_DEBUG.read();
        write_memory_process(data);
    }
    // UART_DEBUG.println("LOOP");
    // delay(1000);
}

