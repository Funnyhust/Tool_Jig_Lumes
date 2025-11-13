# Kế hoạch tích hợp BL0906 Service

## 1. Phân tích Dependencies

### 1.1. Các file/include cần thay thế:

#### Từ project cũ (không có trong project mới):
- `proj/tl_common.h` → **BỎ** (không cần)
- `proj_lib/sig_mesh/app_mesh.h` → **BỎ** (không cần)
- `drivers/8258/i2c.h` → **BỎ** (không dùng I2C)
- `vendor/mesh/user/standard/math_operation.h` → **BỎ** (không cần)
- `vendor/common/mesh_common.h` → **BỎ** (không cần)
- `../debug.h` → **TẠO MỚI** hoặc bỏ (optional)

#### Cần tạo/thay thế:
- `vendor/common/system_time.h` → **THAY** bằng Arduino `millis()`
- `vendor/mesh/user/config_board.h` → **TẠO MỚI** (định nghĩa NUMBER_RL, TIMER_xxx)
- `vendor/mesh/user/utilities.h` → **TẠO MỚI** (macro foreach, BIT, u8, u16, u32)
- `vendor/mesh/user/fifo.h` → **TẠO MỚI** (FIFO đơn giản)

### 1.2. Các hàm cần thay thế:

| Hàm cũ | Thay bằng | File mới |
|--------|-----------|----------|
| `clock_time_ms()` | `millis()` | Arduino.h |
| `clock_time_exceed_ms(t1, timeout)` | Helper function | utilities.h |
| `my_fifo_push_hci_tx_fifo()` | `UartService::writeBytes()` | uart_service.h |
| `FifoInit()` | Tạo FIFO class | fifo.h |
| `FifoPush()` | FIFO class method | fifo.h |
| `FifoPop()` | FIFO class method | fifo.h |
| `FifoIsEmpty()` | FIFO class method | fifo.h |
| `sleep_ms()` | `delay()` | Arduino.h |
| `memcpy()` | `memcpy()` | `<string.h>` (có sẵn) |

### 1.3. Macro/Define cần tạo:

- `u8`, `u16`, `u32`, `s8`, `s32` → typedef trong utilities.h
- `BIT(n)` → `(1 << n)`
- `foreach(i, n)` → `for(uint8_t i = 0; i < n; i++)`
- `NUMBER_RL` → `NUM_RELAYS` (từ relay_service.h)
- `TIMER_500MS` → `500`
- `TIMER_5S` → `5000`

## 2. Cấu trúc file cần tạo

### 2.1. `src/services/bl0906/config_board.h`
```c
#ifndef CONFIG_BOARD_H
#define CONFIG_BOARD_H

#include "../relay/relay_service.h"

#define NUMBER_RL NUM_RELAYS

#endif
```

### 2.2. `src/services/bl0906/utilities.h`
```c
#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>
#include <stdint.h>

// Type definitions
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t s8;
typedef int32_t s32;

// Macros
#define BIT(n) (1 << (n))
#define foreach(i, n) for(uint8_t i = 0; i < (n); i++)

// Time helper
static inline bool clock_time_exceed_ms(uint32_t start_time, uint32_t timeout_ms) {
    return (millis() - start_time) >= timeout_ms;
}

#endif
```

### 2.3. `src/services/bl0906/fifo.h` và `fifo.cpp`
- Tạo class FIFO đơn giản để quản lý command queue

### 2.4. Sửa `bl0906.c`:
- Thay tất cả include từ project cũ
- Thay `my_fifo_push_hci_tx_fifo()` → dùng `UartService`
- Thay `clock_time_ms()` → `millis()`
- Thay `clock_time_exceed_ms()` → helper function
- Thay `sleep_ms()` → `delay()`

## 3. Tích hợp với UART Service

### 3.1. Thay đổi trong `bl0906.c`:
- Thay `my_fifo_push_hci_tx_fifo()` bằng `UartService::writeBytes()`
- Cần truyền `UartService*` vào bl0906 (hoặc dùng singleton)

### 3.2. Nhận dữ liệu từ UART:
- Trong `loop()` hoặc interrupt, gọi `bl0906_handle_serial_rx_message()` khi có data

## 4. Các bước thực hiện

### Bước 1: Tạo utilities.h
- Định nghĩa types, macros, helper functions

### Bước 2: Tạo config_board.h
- Định nghĩa NUMBER_RL

### Bước 3: Tạo fifo.h và fifo.cpp
- Implement FIFO queue đơn giản

### Bước 4: Sửa bl0906.h
- Thay include `../config_board.h` → `config_board.h`
- Sửa các typedef nếu cần

### Bước 5: Sửa bl0906.c
- Xóa các include không cần
- Thay các hàm time, FIFO, UART
- Thay các macro

### Bước 6: Tích hợp vào main.cpp
- Khởi tạo UartService cho BL0906
- Gọi `bl0906_init()` với callback
- Gọi `bl0906_proc()` trong loop()
- Gọi `bl0906_handle_serial_rx_message()` khi nhận UART data

## 5. Lưu ý

- BL0906 giao tiếp qua UART, cần xác định Serial port nào (Serial1, Serial2...)
- Current correction cần tất cả relay đều OFF
- FIFO queue để tránh block khi gửi nhiều command
- Callback function để update giá trị đo được lên application layer

