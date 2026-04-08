// #include "analyze_mode.h"
// #include "../../config.h"
// #include "../uart/uart_service.h"
// #include "../../app/process.h"

// AnalyzeMode::AnalyzeMode(RelayService& relay, UartService* uart) 
//     : _relay(relay), _uart(uart), _enabled(false), _rxIndex(0), _lastRxTime(0), _isBusy(false) {}

// void AnalyzeMode::init() {
//     _enabled = true; 
//     _rxIndex = 0;
//     _isBusy = false;
// }

// extern void delayWithBlink(uint32_t ms);

// void AnalyzeMode::runMonitorCycle() {
//     // 0. RESET DATA - Đưa Dashboard về 0
//     for(int i=0; i<4; i++){
//         bl0906_set_channel_no_reset(i);
//         bl0906_reset_measurements();
//     }
//     broadcastCurrentData();

//     // GIAI ĐOẠN 1: BẬT FULL CHANNEL (3 RELAY CÙNG LÚC)
//     for (int ch = 0; ch < 4; ch++) {
//         // BẬT CẢ 3 RELAY
//         for(int j=0; j<3; j++) _relay.turnOn(ch * 3 + j);
//         delayWithBlink(1500); // Chờ ổn định tải

//         // ĐỌC 5 LẦN & GỬI NGAY LÊN APP
//         for (int j = 0; j < 5; j++) {
//             process_analyze_update_all();
//             broadcastCurrentData();
//             delayWithBlink(500); 
//         }

//         // TẮT RELAY
//         for(int j=0; j<3; j++) _relay.turnOff(ch * 3 + j);
//         delayWithBlink(200);
//     }

//     // GIAI ĐOẠN 2: BẬT TỪNG RELAY LẺ (INDIVIDUAL)
//     for (int ch = 0; ch < 4; ch++) {
//         for (int r = 0; r < 3; r++) {
//             _relay.turnOn(ch * 3 + r);
//             delayWithBlink(1000);

//             process_analyze_update_all();
//             broadcastCurrentData();

//             _relay.turnOff(ch * 3 + r);
//             delayWithBlink(200);
//         }
//     }

//     // KẾT THÚC: REPORT 0 LÊN DASHBOARD
//     for(int i=0; i<4; i++) {
//         bl0906_set_channel_no_reset(i);
//         bl0906_reset_measurements();
//     }
//     broadcastCurrentData();
// }

// void AnalyzeMode::autoBroadcast() {
//     static uint32_t lastBroadcast = 0;
//     if (millis() - lastBroadcast >= 500) {
//         lastBroadcast = millis();
        
//         // Nếu chuyển sang trạng thái Rảnh (không chạy process), chúng ta Fetch mới UART
//         // Điều này giúp Dashboard về 0 ngay lập tức khi Relay ngắt
//         if (!_isBusy) {
//             process_analyze_update_all();
//         }
        
//         broadcastCurrentData();
//     }
// }

// void AnalyzeMode::broadcastCurrentData() {
//     uint8_t buffer[72]; 
//     memset(buffer, 0, sizeof(buffer));

//     for (int ch = 0; ch < 4; ch++) {
//         bl0906_set_channel_no_reset(ch);
//         measurement_value_t m = bl0906_get_all_measurements();
        
//         uint16_t v_val = (uint16_t)(m.voltage * 10);
//         buffer[ch*2] = (v_val >> 8) & 0xFF;
//         buffer[ch*2 + 1] = v_val & 0xFF;

//         for (int i = 0; i < 3; i++) {
//             int idx = ch * 3 + i;
//             uint16_t i_val = (uint16_t)m.current[i];
//             buffer[8 + idx*2] = (i_val >> 8) & 0xFF;
//             buffer[9 + idx*2] = i_val & 0xFF;
            
//             uint16_t p_val = (uint16_t)(m.active_power[i] * 10);
//             buffer[32 + idx*2] = (p_val >> 8) & 0xFF;
//             buffer[33 + idx*2] = p_val & 0xFF;
//         }
        
//         uint32_t g_val = m.gain;
//         buffer[56 + ch*4] = (g_val >> 24) & 0xFF;
//         buffer[57 + ch*4] = (g_val >> 16) & 0xFF;
//         buffer[58 + ch*4] = (g_val >> 8) & 0xFF;
//         buffer[59 + ch*4] = g_val & 0xFF;
//     }
//     // sendResponse wraps data with Header, Cmd, Checksum
//     sendResponse(CMD_READ_ALL_DATA, buffer, 72);
// }

// void AnalyzeMode::process(uint8_t byte) {
//     if (byte == ANALYZE_HEADER) {
//         _rxIndex = 0;
//         _rxBuffer[_rxIndex++] = byte;
//         _lastRxTime = millis();
//         return;
//     }
//     if (_rxIndex > 0 && (millis() - _lastRxTime < 500)) {
//         _rxBuffer[_rxIndex++] = byte;
//         _lastRxTime = millis();
//         if (_rxIndex == 5) {
//             handleCommand();
//             _rxIndex = 0;
//         }
//     }
// }

// void AnalyzeMode::handleCommand() {
//     uint8_t cmd = _rxBuffer[1];
//     uint8_t d1 = _rxBuffer[2];
//     uint8_t d2 = _rxBuffer[3];
//     uint8_t checksum = _rxBuffer[4];
//     uint8_t expectedCS = (ANALYZE_HEADER + cmd + d1 + d2) & 0xFF;
//     if (checksum != expectedCS) return;

//     if (cmd == CMD_ENABLE_ANALYZE) {
//         _enabled = true;
//         sendResponse(CMD_ENABLE_ANALYZE, _rxBuffer + 2, 2);
//     } else if (cmd == CMD_DISABLE_ANALYZE) {
//         _enabled = false;
//         sendResponse(CMD_DISABLE_ANALYZE, _rxBuffer + 2, 2);
//     } else if (_enabled) {
//         if (cmd == CMD_READ_ALL_DATA) {
//             sendAllData();
//         } else if (cmd >= CMD_RELAY_CONTROL && cmd < CMD_RELAY_CONTROL + NUM_RELAYS) {
//             uint8_t relayIdx = cmd - CMD_RELAY_CONTROL;
//             if (d1 == 0x01) _relay.turnOn(relayIdx);
//             else _relay.turnOff(relayIdx);
//             uint8_t respData[2] = { _relay.isOn(relayIdx) ? (uint8_t)0x01 : (uint8_t)0x00, 0x00 };
//             sendResponse(cmd, respData, 2);
//         } else if (cmd == CMD_BATCH_CONTROL) {
//             uint16_t mask = (d1 << 8) | d2;
//             for (int i = 0; i < NUM_RELAYS; i++) {
//                 if (mask & (1 << i)) _relay.turnOn(i);
//                 else _relay.turnOff(i);
//             }
//             sendResponse(CMD_BATCH_CONTROL, _rxBuffer + 2, 2);
//         }
//     }
// }

// void AnalyzeMode::sendResponse(uint8_t cmd, uint8_t* data, uint8_t len) {
//     uint8_t header = ANALYZE_HEADER;
//     HardwareSerial* serial = _uart->getSerial();
//     if (!serial) return;
//     serial->write(header);
//     serial->write(cmd);
//     uint8_t cs = header + cmd;
//     for (uint8_t i = 0; i < len; i++) {
//         serial->write(data[i]);
//         cs += data[i];
//     }
//     serial->write(cs);
// }

// void AnalyzeMode::sendAllData() {
//     process_analyze_update_all();
//     broadcastCurrentData();
// }
