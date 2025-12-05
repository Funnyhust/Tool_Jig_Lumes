#include "process.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdio.h>
#include "../services/bl0906/bl0906.h"
#include "../services/uart/uart_service.h"
#include "../services/relay/relay_service.h"
#include "../services/zero_detect/zero_detect.h"
#include "../config.h"
#include "../services/eeprom_at24c02/at24c02.h"
// Khai báo hàm delay có blink LED từ main.cpp


//Note hardware
//M2 SCL
//M3 SDA



extern void delayWithBlink(uint32_t ms);

// 4 uart bl0906, 1 uart debug
// Khởi tạo trong hàm để tránh lỗi undefined reference
static UartService* uartBl0906[4] = {NULL, NULL, NULL, NULL};
static UartService* uartDebug = NULL;


// Lưu giá trị đo được cho từng kênh BL0906
static measurement_value_t channel_measurements[4];

uint32_t voltage_sum_uv[4] = {0, 0, 0, 0};
uint32_t current_sum_ua[4][3] = {0, 0, 0, 0};
uint32_t power_sum_uw[4][3] = {0, 0, 0, 0};

// Đếm số lần đo được cho từng kênh BL0906
uint8_t voltage_calib_count[4] = {0, 0, 0, 0};
uint8_t current_calib_count[4][3] = {{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0}};
uint8_t power_calib_count[4][3] = {{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0}};


uint32_t voltage_calib_value[4] = {0, 0, 0, 0};
//đơn vị uV 4 channel, 3 current
uint32_t current_calib_value[4][3] = {{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0}};
uint32_t power_calib_value[4][3] = {{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0}};

//hệ số calibration
uint16_t kVoltage[4] ;
uint16_t kCurrent[4][3];
uint16_t kPower[4][3];

uint8_t eeprom_value[4][16];

// Giá trị ngưỡng mặc định (đơn vị như comment trong process.h)
const uint32_t VOLTAGE_THRESHOLD_DEFAULT[4] = {
    220000, 220000, 220000, 220000   // mV
};

const uint32_t CURRENT_THRESHOLD_DEFAULT[4][3] = {
    {182800, 185300, 185500},        // kênh 0
    {185000, 185000, 185000},        // kênh 1
    {185000, 185000, 185000},        // kênh 2
    {185000, 185000, 185000}         // kênh 3
}; // mA

const uint32_t POWER_THRESHOLD_DEFAULT[4][3] = {
    {40330, 40730, 40810},           // kênh 0
    {40780, 40670, 41410},           // kênh 1
    {40470, 40540, 40890},           // kênh 2
    {40420, 41100, 40710}            // kênh 3
}; // mW





static void process_calibrate(){
    for(int i = 0; i < 4; i++){
        if(voltage_calib_count[i] > 0){
            voltage_calib_value[i] = voltage_sum_uv[i] / voltage_calib_count[i];
            kVoltage[i] = voltage_calib_value[i] / VOLTAGE_THRESHOLD_DEFAULT[i];
        }
    }
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 3; j++){
            if(current_calib_count[i][j] > 0){
                current_calib_value[i][j] = current_sum_ua[i][j] / current_calib_count[i][j];
                kCurrent[i][j] = current_calib_value[i][j] / CURRENT_THRESHOLD_DEFAULT[i][j];
            }
        }
    }
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 3; j++){
            if(power_calib_count[i][j] > 0){
                power_calib_value[i][j] = power_sum_uw[i][j] / power_calib_count[i][j];
                kPower[i][j] = power_calib_value[i][j] / POWER_THRESHOLD_DEFAULT[i][j];
            }
        }
    }
}


void prepare_wrire_eeprom_value(void){
    for(int i = 0; i < 4; i++){
        eeprom_value[i][0] = kVoltage[i];
        eeprom_value[i][1] = kVoltage[i] >> 8;
        for(int j = 0; j < 3; j++){
            eeprom_value[i][j*2+2] = kCurrent[i][j];  
            eeprom_value[i][j*2+2+1] = kCurrent[i][j] >> 8;
        }
        for(int j = 0; j < 3; j++){
            eeprom_value[i][j*2+8] = kPower[i][j];
            eeprom_value[i][j*2+8+1] = kPower[i][j] >> 8;
        }
        //2 byte cuối bằng 0
        eeprom_value[i][14] = 0;
        eeprom_value[i][15] = 0;
    }


}

static uint8_t page_to_write[8];
static uint8_t block_to_read[14];
static bool write_eeprom_success[4] = {false, false, false, false};
void write_eeprom_value(void){
    for(int ch = 0; ch < 4; ch++){
        at24c02_init(ch+1);
        delayWithBlink(10);
        for(int j = 0; j < 3; j++){ // retry max 3 times
            //Ghi page địa chỉ 0x00 - 0x07  
            for(int k = 0; k < 8; k++){
                page_to_write[k] = eeprom_value[ch][k];
            }
            at24c02_write_block(0x00, page_to_write, 8);
            delayWithBlink(50);
            //Ghi page địa chỉ 0x08 - 0x0F
            for(int k = 0; k < 8; k++){
                page_to_write[k] = eeprom_value[ch][k+8];
                }
            at24c02_write_block(0x08, page_to_write, 8);
            delayWithBlink(50);
            //Đọc page địa chỉ 0x00 - 0x0D
            at24c02_read_block(0x00, block_to_read, 14);
            //Kiểm tra giá trị đọc được có giống với giá trị ghi vào không
            bool ok = true;
            for(int k = 0; k < 14; k++){
                if(block_to_read[k] != eeprom_value[ch][k]){
                    ok = false;
                    break;
                }
            }
            if(ok){
                //Nếu ghi thành công thì không retry nữa
                write_eeprom_success[ch] = true;
                break;
            }
            }
        }
    }

void start_write_eeprom_value(void){    
       process_calibrate();
       prepare_wrire_eeprom_value();
       write_eeprom_value();
}

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

void check_channel_pass(int channel){
    //kiểm tra theo giá trị calib
    if((kVoltage[channel] <900) || (kVoltage[channel] > 1100)){
        channel_measurements[channel].voltage_ok = false;
    }
    else {
        channel_measurements[channel].voltage_ok = true;
    }
    if((kCurrent[channel][0] <900) || (kCurrent[channel][0] > 1100)){
        channel_measurements[channel].current_1_ok = false;
    }
    else {
        channel_measurements[channel].current_1_ok = true;
    }
    if((kCurrent[channel][1] <900) || (kCurrent[channel][1] > 1100)){
        channel_measurements[channel].current_2_ok = false;
    }
    else {
        channel_measurements[channel].current_2_ok = true;
    }
    if((kCurrent[channel][2] <900) || (kCurrent[channel][2] > 1100)){
        channel_measurements[channel].current_3_ok = false;
    }
    else {
        channel_measurements[channel].current_3_ok = true;
    }
    if((kPower[channel][0] <900) || (kPower[channel][0] > 1100)){
        channel_measurements[channel].power_1_ok = false;
    }
    else {
        channel_measurements[channel].power_1_ok = true;
    }
    if((kPower[channel][1] <900) || (kPower[channel][1] > 1100)){
        channel_measurements[channel].power_2_ok = false;
    }
    else {
        channel_measurements[channel].power_2_ok = true;
    }
    if((kPower[channel][2] <900) || (kPower[channel][2] > 1100)){
        channel_measurements[channel].power_3_ok = false;
    }
    else {
        channel_measurements[channel].power_3_ok = true;
    }
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

    // Lưu giá trị vào mảng của kênh này ngay sau khi đọc giá trị đơn vị uV, pA, uW
    channel_measurements[channel] = bl0906_get_all_measurements(); 
    if (channel_measurements[channel].voltage > 0) {
    voltage_sum_uv[channel] += channel_measurements[channel].voltage*1000*1000;
    voltage_calib_count[channel]++;
    }
    if (channel_measurements[channel].current[0] > 0) {     
        current_sum_ua[channel][0] += channel_measurements[channel].current[0]*1000*1000;
        current_calib_count[channel][0]++;
    }
    if (channel_measurements[channel].current[1] > 0) {
        current_sum_ua[channel][1] += channel_measurements[channel].current[1]*1000*1000;
        current_calib_count[channel][1]++;
    }
    if (channel_measurements[channel].current[2] > 0) {
        current_sum_ua[channel][2] += channel_measurements[channel].current[2]*1000*1000;
        current_calib_count[channel][2]++;
    }
    if (channel_measurements[channel].active_power[0] > 0) {
        power_sum_uw[channel][0] += channel_measurements[channel].active_power[0]*1000*1000 ;
        power_calib_count[channel][0]++;
    }
    if (channel_measurements[channel].active_power[1] > 0) {
        power_sum_uw[channel][1] += channel_measurements[channel].active_power[1]*1000*1000;
        power_calib_count[channel][1]++;
    }
    if (channel_measurements[channel].active_power[2] > 0) {
        power_sum_uw[channel][2] += channel_measurements[channel].active_power[2]*1000*1000;
        power_calib_count[channel][2]++;
    }
}




// Khai báo biến điều khiển LED từ main.cpp
extern volatile bool ledBlinkEnable;

void start_process(void)
{
  // turn on all relay
   uint32_t start_time_ms = millis();
   // turn on all relay
   relayService.turnOnAll();
   zero_detect_process();
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
   for (int j = 0; j < 5; j++) {
        uint32_t calib_start_time_ms = millis();
        if (millis() - calib_start_time_ms > 1000) {
            break;
        }
        for (int i = 0; i < 4; i++) {
                read_bl0906_channel(i, uartBl0906[i]);
                delayWithBlink(2);    
        }
        if(j<4){
        while (millis() - calib_start_time_ms < 650) {
            delayWithBlink(5);
        }
        }
  }

  start_write_eeprom_value();
  
   // Kiểm tra kết quả và điều khiển relay
   for (int i = 0; i < 4; i++) {
       is_channel_pass(i);
       // Kiểm tra zero detect: nếu không pass thì tắt tất cả relay của kênh đó
       bool zero_ok = zero_detect_get_result(i);
       if(!zero_ok) {
           // Zero detect fail - tắt cả 3 relay của kênh này
        //    UART_DEBUG.print("Zero detect fail for channel: ");
        //    UART_DEBUG.println(i);
        //    zero_detect_result[i] = false;
        //    relayService.setRelayState(i*3, false);
        //    relayService.setRelayState(i*3+1, false);
        //    relayService.setRelayState(i*3+2, false);
       } else {
        check_channel_pass(i);
        if(!channel_measurements[i].voltage_ok){
            relayService.setRelayState(i*3, false);
            relayService.setRelayState(i*3+1, false);
            relayService.setRelayState(i*3+2, false);
        }
        else {
            relayService.setRelayState(i*3, channel_measurements[i].current_1_ok&channel_measurements[i].power_1_ok);
            relayService.setRelayState(i*3+1, channel_measurements[i].current_2_ok&channel_measurements[i].power_2_ok);
            relayService.setRelayState(i*3+2, channel_measurements[i].current_3_ok&channel_measurements[i].power_3_ok); 
        }
       }
   }
    //Write eeprom status
    for(int i = 0; i < 4; i++){
        UART_DEBUG.print("Write eeprom is:");
        UART_DEBUG.print(write_eeprom_success[i] ? "Success" : "Fail");
        UART_DEBUG.print(" for channel: ");
        UART_DEBUG.println(i);
    }

    //Print kVoltage, kCurrent, kPower
    UART_DEBUG.print("kVoltage: ");
    UART_DEBUG.println(kVoltage[0]);
    UART_DEBUG.print("kCurrent: ");
    UART_DEBUG.println(kCurrent[0][0]);
    UART_DEBUG.print("kPower: ");
    UART_DEBUG.println(kPower[0][0]);
    UART_DEBUG.println("Process done");
    UART_DEBUG.print("Time process: ");
    UART_DEBUG.println(millis() - start_time_ms);
}

