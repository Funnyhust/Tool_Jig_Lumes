#include "control_power.h"
#include <Arduino.h>
#include "HardwareTimer.h"
#include "config.h"

/* ================= CONFIG ================= */

#define CONFIRM_TIME_US   50000UL   // 50 ms

/* ================= INTERNAL ================= */

// Dùng TIM3
static HardwareTimer *powerSafetyTimer = nullptr;
static volatile bool timerArmed = false;

/* ================= ISR ================= */

//================Log time================//
uint32_t log_time_start = 0;
uint32_t log_time_end = 0;
//========================================//


void control_power_on(void)
{
    digitalWrite(POWER_CONTROL_PIN, HIGH);
}

/* TẮT nguồn */
void control_power_shutdown(void)
{
    digitalWrite(POWER_CONTROL_PIN, LOW);
}



// GPIO interrupt: chỉ arm timer
void power_control_gpio_isr(void)
{
    log_time_start = millis();
    if (!timerArmed) {
        timerArmed = true;
        powerSafetyTimer->setCount(0);
        powerSafetyTimer->resume();   // start one-shot
    }
}

// Timer interrupt: xác nhận & cắt nguồn
void power_control_timer_isr(void)
{
    powerSafetyTimer->pause();   // one-shot

    if (digitalRead(POWER_CONTROL_SIGNAL_PIN) == HIGH) {
        // FAIL SAFE: TẮT NGUỒN
        control_power_shutdown();

    }
    log_time_end = millis();
    UART_DEBUG.print("Power control: Timer triggered - shutting down power. Duration: ");
    UART_DEBUG.print(log_time_end - log_time_start);
    UART_DEBUG.println(" ms");

    timerArmed = false;
}

/* ================= PUBLIC API ================= */

void control_power_init(void)
{
    // Công tắc NC
    pinMode(POWER_CONTROL_SIGNAL_PIN, INPUT_PULLUP);

    // Điều khiển nguồn (LOW = OFF, HIGH = ON)
    pinMode(POWER_CONTROL_PIN, OUTPUT);
    digitalWrite(POWER_CONTROL_PIN, LOW);

    // GPIO interrupt: nhấc công tắc
    attachInterrupt(
        digitalPinToInterrupt(POWER_CONTROL_SIGNAL_PIN),
        power_control_gpio_isr,
        RISING
    );

    // ===== TIMER SETUP (STM32) =====
    powerSafetyTimer = new HardwareTimer(TIM3);

    powerSafetyTimer->setPrescaleFactor(72); 
    // 72 MHz / 72 = 1 MHz → 1 tick = 1 µs

    powerSafetyTimer->setOverflow(CONFIRM_TIME_US, MICROSEC_FORMAT);
    powerSafetyTimer->attachInterrupt(power_control_timer_isr);
    powerSafetyTimer->pause();   // chỉ start khi có interrupt
}

/* Công tắc đang được giữ? */
bool is_return_power_control_signal(void)
{
    UART_DEBUG.print("Power control signal state: ");
    if (digitalRead(POWER_CONTROL_SIGNAL_PIN) == LOW) {
        UART_DEBUG.println("LOW (ON)");
        return true;
    } else {
        UART_DEBUG.println("HIGH (OFF)");
        return false;
    }
    return (digitalRead(POWER_CONTROL_SIGNAL_PIN) == LOW);
}



/* BẬT nguồn */
