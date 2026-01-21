#include "app/process.h"
#include "config.h"
#include "services/control_power/control_power.h"
#include "services/eeprom_at24c02/at24c02.h"
#include "services/relay/relay_service.h"
#include "services/write_memory/write_memory.h"
#include "services/zero_detect/zero_detect.h"
#include "stm32f1xx_hal.h"
#include <Arduino.h>
#include <HardwareSerial.h>

RelayService relayService;
uint32_t time_end_process = 0;
bool is_first_run = true;

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
      if (currentTime - ledLastBlinkTime >= 200) { // Blink mỗi 200ms
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        ledLastBlinkTime = currentTime;
      }
    }
    delay(10); // Delay nhỏ để không chiếm CPU
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
  switch (sysclk_source) {
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

void setup() {
  debug_init();
  pinMode(LED_PIN, OUTPUT);
  relayService.init();
  zero_detect_init_pin();
  control_power_init();
  digitalWrite(LED_PIN, HIGH);
  delay(500);
}

void loop() {
    if (is_return_power_control_signal()) {
      if (is_first_run) {
      
      UART_DEBUG.println("Power control signal detected");
      delay(2000);
      control_power_on();
      is_first_run = false;
      process_init();
      ledBlinkEnable = true;
      start_process();
      ledBlinkEnable = false;
      digitalWrite(LED_PIN, LOW);
    }
  }
  //Nếu thời gian quá 5s thì reset is first run
  if (is_first_run && millis() - time_end_process >= 5000) {
    is_first_run = true;
  }

  if (millis() - time_end_process >= 30000) {
    relayService.turnOffAll();
  }
  if (UART_DEBUG.available()) {
    uint8_t data = UART_DEBUG.read();
    write_memory_process(data);
  }
}
