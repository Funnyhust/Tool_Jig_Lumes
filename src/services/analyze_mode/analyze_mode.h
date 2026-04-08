#ifndef ANALYZE_MODE_H
#define ANALYZE_MODE_H

#include <Arduino.h>
#include "../relay/relay_service.h"
#include "../bl0906/bl0906.h"

#define ANALYZE_HEADER 0xAA

// Command IDs
#define CMD_ENABLE_ANALYZE  0x01
#define CMD_DISABLE_ANALYZE 0x00
#define CMD_READ_ALL_DATA   0x10
#define CMD_RELAY_CONTROL   0x20 // 0x20 + RelayIndex (0-11)
#define CMD_BATCH_CONTROL   0x30

class AnalyzeMode {
public:
    AnalyzeMode(RelayService& relay, UartService* uart);
    void init();
    void process(uint8_t byte);
    void runMonitorCycle();         // Quy trình test mô phỏng để Monitor
    void autoBroadcast();           // Thông minh: Tự quyết định Fetch+Send hoặc chỉ Send
    void broadcastCurrentData();    // Chỉ Gửi (Dùng nội bộ)
    void setBusy(bool busy) { _isBusy = busy; }
    bool isEnabled() const { return _enabled; }
    bool isAnalyzePacket(uint8_t byte) { return byte == ANALYZE_HEADER; }
    bool isProcessingPacket() const { return _rxIndex > 0; }

private:
    void handleCommand();
    void sendResponse(uint8_t cmd, uint8_t* data, uint8_t len);
    void sendAllData();
    uint8_t calculateChecksum(uint8_t cmd, uint8_t* data, uint8_t len);

    RelayService& _relay;
    UartService* _uart;
    bool _enabled;
    bool _isBusy;
    
    uint8_t _rxBuffer[8];
    uint8_t _rxIndex;
    uint32_t _lastRxTime;
};

#endif // ANALYZE_MODE_H
