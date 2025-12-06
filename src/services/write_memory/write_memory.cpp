#include "write_memory.h"
#include "../../config.h"

#define RESET_SETTING_BUFFER_TIME 1000

// Định nghĩa các giá trị default (chỉ định nghĩa ở đây, không phải trong header)
uint32_t FIRST_WRITE_VOLTAGE_THRESHOLD[4] = {
    220000, 220000, 220000, 220000   // mV
};

uint32_t FIRST_WRITE_CURRENT_THRESHOLD[4][3] = {
    {182800, 185300, 185500},        // kênh 0
    {185400, 184600, 188200},        // kênh 1
    {184700, 184000, 185600},        // kênh 2
    {183700, 186200, 185200}         // kênh 3
}; // mA

uint32_t FIRST_WRITE_POWER_THRESHOLD[4][3] = {
    {40330, 40730, 40810},           // kênh 0
    {40780, 40670, 41410},           // kênh 1
    {40470, 40540, 40890},           // kênh 2
    {40420, 41100, 40710}            // kênh 3
}; // mW

// Biến để đánh dấu đã khởi tạo chưa
write_memory_t setting_data;

static bool memory_initialized = false;


// Địa chỉ bắt đầu của mỗi channel trong EEPROM
static const uint16_t MEMORY_ADDR_CHANNEL[4] = {
    MEMORY_ADDR_CHANNEL_0,
    MEMORY_ADDR_CHANNEL_1,
    MEMORY_ADDR_CHANNEL_2,
    MEMORY_ADDR_CHANNEL_3
};

static uint8_t rx_setting_buffer[17];
static uint8_t rx_setting_buffer_index = 0;
static bool check_setting_buffer_full = false;
    
static void reset_setting_buffer(void){
    rx_setting_buffer_index = 0;
    check_setting_buffer_full = false;
}

uint32_t time_rx_setting_buffer = 0;
uint32_t last_time_rx_setting_buffer = 0;
static void add_to_setting_buffer(uint8_t data){
    time_rx_setting_buffer = millis();
    if(time_rx_setting_buffer - last_time_rx_setting_buffer > RESET_SETTING_BUFFER_TIME){
        reset_setting_buffer();
    }
    rx_setting_buffer[rx_setting_buffer_index] = data;
    rx_setting_buffer_index++;
    if(rx_setting_buffer_index == 17){
        check_setting_buffer_full = true;
    }
    last_time_rx_setting_buffer = time_rx_setting_buffer;
}

static bool calculate_checksum(void){
    uint8_t checksum = 0;
    for(int i = 1; i < 14; i++){
        checksum += rx_setting_buffer[i];
    }
    checksum = ~checksum;  // Sửa syntax: phải dùng = thay vì ~=
    UART_DEBUG.print("Checksum: ");
    UART_DEBUG.println(rx_setting_buffer[16]);
    UART_DEBUG.print("Checksum expected: ");
    UART_DEBUG.println(checksum);
    return checksum == rx_setting_buffer[16];
}

static bool check_valid_frame(void){
    // Kiểm tra frame ID - phải bằng 0xD3
    if(rx_setting_buffer[0] != SETTING_FRAME_ID){
        UART_DEBUG.print("Invalid frame ID. Expected: 0xD3, Received: 0x");
        UART_DEBUG.println(rx_setting_buffer[0], HEX);
        return false;
    }
    // Kiểm tra checksum
    if(!calculate_checksum()){
        UART_DEBUG.println("Invalid checksum");
        return false;
    }
    return true;
}

static bool mapping_process(){
    if(!check_valid_frame()){
        UART_DEBUG.print("Invalid frame");
        return false;
    }
    static uint32_t data = 0;
    setting_data.channel = rx_setting_buffer[1];
    data= (rx_setting_buffer[2]<<8 | rx_setting_buffer[3])*1000;
    setting_data.voltage_threshold = data;
    data= (rx_setting_buffer[4]<<8 | rx_setting_buffer[5])*100;
    setting_data.current_threshold[setting_data.channel][0] = data;
    data= (rx_setting_buffer[6]<<8 | rx_setting_buffer[7])*100;
    setting_data.current_threshold[setting_data.channel][1] = data;
    data= (rx_setting_buffer[8]<<8 | rx_setting_buffer[9])*100;
    setting_data.current_threshold[setting_data.channel][2] = data;
    data= (rx_setting_buffer[10]<<8 | rx_setting_buffer[11]);
    setting_data.power_threshold[setting_data.channel][0] = data;
    data= (rx_setting_buffer[12]<<8 | rx_setting_buffer[13]);
    setting_data.power_threshold[setting_data.channel][1] = data;
    data= (rx_setting_buffer[14]<<8 | rx_setting_buffer[15]);
    setting_data.power_threshold[setting_data.channel][2] = data;
    return true;
}

static void write_memory_init(void) {
    if (memory_initialized) {
        return;
    }
    
    // Khởi tạo EEPROM emulated (Arduino dùng Flash của STM32 để mô phỏng EEPROM)
    // Đây KHÔNG phải EEPROM ngoài AT24C02 - đó là module riêng trong at24c02.h/cpp
    // Trên STM32, EEPROM.begin() không nhận tham số size
    EEPROM.begin();
    memory_initialized = true;
}

/**
 * @brief Ghi dữ liệu threshold của một channel vào EEPROM
 * @param channel Channel number (0-3)
 * @note Dùng EEPROM.put() để ghi cả uint32_t một lúc, không cần ghi từng byte
 */
static void write_memory_to_eeprom_channel(uint8_t channel){
    if (!memory_initialized) {
        write_memory_init();
    }
    
    if (channel >= 4) {
        return; // Invalid channel
    }
    
    uint16_t base_addr = MEMORY_ADDR_CHANNEL[channel];
    
    // Ghi voltage_threshold (4 bytes) - dùng EEPROM.put() để ghi cả uint32_t
    EEPROM.put(base_addr, setting_data.voltage_threshold);
    
    // Ghi current_threshold[3] (3 * 4 = 12 bytes)
    // Có thể ghi từng giá trị hoặc ghi cả mảng 3 phần tử
    for (uint8_t i = 0; i < 3; i++) {
        EEPROM.put(base_addr + 4 + i * sizeof(uint32_t), setting_data.current_threshold[channel][i]);
    }
    
    // Ghi power_threshold[3] (3 * 4 = 12 bytes)
    for (uint8_t i = 0; i < 3; i++) {
        EEPROM.put(base_addr + 4 + 12 + i * sizeof(uint32_t), setting_data.power_threshold[channel][i]);
    }
    
    // Trên STM32, EEPROM tự động commit, không cần gọi EEPROM.commit()
}


//Clear memory for channel i
static void write_memory_clear_channel(uint8_t channel){
    if (!memory_initialized) {
        write_memory_init();
    }
    
    // Xóa tất cả dữ liệu của channel (ghi 0xFF)
    uint16_t base_addr = MEMORY_ADDR_CHANNEL[channel];
    for (uint16_t i = 0; i < 28; i++) {
        EEPROM.write(base_addr + i, 0xFF);
    }
    
    // Trên STM32, EEPROM tự động commit, không cần gọi EEPROM.commit()
    UART_DEBUG.print("Write memory: Cleared channel ");
    UART_DEBUG.println(channel);    
}


void write_memory_process(uint8_t byte){
    add_to_setting_buffer(byte);
    if(check_setting_buffer_full){
        if(!mapping_process()){
            reset_setting_buffer();
            return;
        }
        // Ghi dữ liệu vào EEPROM cho channel này
        write_memory_to_eeprom_channel(setting_data.channel);
        reset_setting_buffer();
        UART_DEBUG.print("Write memory success");
    }

}

/**
 * @brief Đọc dữ liệu threshold của một channel từ EEPROM
 * @param channel Channel number (0-3)
 * @param voltage_threshold Con trỏ để lưu voltage threshold (4 bytes)
 * @param current_threshold Con trỏ để lưu current threshold array (3 * 4 = 12 bytes)
 * @param power_threshold Con trỏ để lưu power threshold array (3 * 4 = 12 bytes)
 * @return true nếu đọc thành công, false nếu channel không hợp lệ
 */
bool read_eeprom_channel(uint8_t channel, 
                                uint32_t* voltage_threshold,
                                uint32_t* current_threshold,
                                uint32_t* power_threshold) {
    if (!memory_initialized) {
        write_memory_init();
    }
    
    if (channel >= 4 || voltage_threshold == NULL || 
        current_threshold == NULL || power_threshold == NULL) {
        return false; // Invalid channel or NULL pointer
    }
    
    uint16_t base_addr = MEMORY_ADDR_CHANNEL[channel];
    
    // Đọc voltage_threshold (4 bytes) - dùng EEPROM.get() để đọc cả uint32_t
    EEPROM.get(base_addr, *voltage_threshold);
    
    // Đọc current_threshold[3] (3 * 4 = 12 bytes)
    for (uint8_t i = 0; i < 3; i++) {
        EEPROM.get(base_addr + 4 + i * sizeof(uint32_t), current_threshold[i]);
    }
    
    // Đọc power_threshold[3] (3 * 4 = 12 bytes)
    for (uint8_t i = 0; i < 3; i++) {
        EEPROM.get(base_addr + 4 + 12 + i * sizeof(uint32_t), power_threshold[i]);
    }
    
    return true;
}

/**
 * @brief Đọc tất cả các giá trị threshold từ EEPROM cho tất cả 4 channels
 * @param voltage_threshold Mảng 4 phần tử để lưu voltage threshold
 * @param current_threshold Mảng 4x3 để lưu current threshold
 * @param power_threshold Mảng 4x3 để lưu power threshold
 * @return true nếu đọc thành công ít nhất 1 channel, false nếu không
 */
bool read_all_eeprom_channels(uint32_t voltage_threshold[4],
                                     uint32_t current_threshold[4][3],
                                     uint32_t power_threshold[4][3]) {
    bool success = false;
    
    for (uint8_t ch = 0; ch < 4; ch++) {
        if (read_eeprom_channel(ch, 
                                       &voltage_threshold[ch],
                                       current_threshold[ch],
                                       power_threshold[ch])) {
            success = true;
        }
    }
    
    return success;
}

void first_write_memory_all_channels(void){
    //Dùng giá trị default để ghi vào EEPROM
    for(uint8_t ch = 0; ch < 4; ch++){
        setting_data.channel = ch;
        setting_data.voltage_threshold = FIRST_WRITE_VOLTAGE_THRESHOLD[ch];
        for(uint8_t i = 0; i < 3; i++){
            setting_data.current_threshold[ch][i] = FIRST_WRITE_CURRENT_THRESHOLD[ch][i];
        }
        for(uint8_t i = 0; i < 3; i++){
            setting_data.power_threshold[ch][i] = FIRST_WRITE_POWER_THRESHOLD[ch][i];
        }
        write_memory_to_eeprom_channel(ch);
    }
} 

