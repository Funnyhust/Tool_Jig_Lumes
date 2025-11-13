# Quy trình READ của BL0906

## Tổng quan
BL0906 sử dụng **FIFO Queue** để quản lý các lệnh đọc register, tránh block và xử lý tuần tự.

## Quy trình READ (Step by Step)

### **BƯỚC 1: Yêu cầu đọc (Application Layer)**
```
bl0906_send_get_current() 
  → bl0906_read_register(I1_RMS)
  → bl0906_read_register(I2_RMS)  
  → bl0906_read_register(I3_RMS)
```

**Hoặc:**
```
bl0906_get_voltage()
  → bl0906_read_register(V_RMS)
```

### **BƯỚC 2: Push vào FIFO Queue**
```c
bl0906_read_register(u8 address)
  → bl0906_push_wait_reg_rsp_to_fifo(address)
    → Tạo bl0906_read_cmd_t { id_register = address }
    → FifoPush(&fifo_bl0906_cmd, &cmd)  // Đưa vào hàng đợi
```

**Lưu ý:** Nhiều lệnh có thể được push vào FIFO, sẽ xử lý tuần tự.

### **BƯỚC 3: Xử lý FIFO (gọi trong bl0906_proc())**
```c
bl0906_fifo_proc()
```

**Logic:**
1. **Kiểm tra command đang chạy:**
   - Nếu `cmd_is_running.id_register != REG_UNKNOWN`:
     - Kiểm tra timeout (100ms)
     - Nếu timeout → reset về REG_UNKNOWN (bỏ qua command này)
   
2. **Nếu không có command đang chạy:**
   - Kiểm tra FIFO có rỗng không
   - Nếu có command trong FIFO:
     - `FifoPop()` → lấy command ra
     - Lưu vào `cmd_is_running`
     - Ghi thời gian bắt đầu: `active_st_time = clock_time_ms()`
     - **Gán callback handler** dựa trên register ID:
       ```
       I1_RMS, I2_RMS, I3_RMS → bl0906_handle_current_rsp
       V_RMS                  → bl0906_handle_voltage_rsp
       WATT_1, WATT_2, WATT_3  → bl0906_handle_active_power_rsp
       TPS                    → bl0906_handle_temperature_rsp
       RMSOS_1, RMSOS_2, RMSOS_3 → bl0906_handle_rmsos_rsp
       GAIN_1                 → bl0906_handle_gain_rsp
       ```
     - **Gửi command qua UART:**
       ```c
       u8 tx_data[] = { BL0906_READ_COMMAND (0x35), register_id };
       bl0906_push_msg(tx_data, 2);  // Gửi: [0x35, register_id]
       ```

### **BƯỚC 4: Nhận response từ BL0906 chip**
Khi có data từ UART, gọi:
```c
bl0906_handle_serial_rx_message(u8* buff, u8 len)
```

**Logic xử lý:**
1. **Kiểm tra có command đang chờ không:**
   - Nếu `cmd_is_running.id_register == REG_UNKNOWN` → bỏ qua
   
2. **Kiểm tra checksum:**
   ```c
   checksum = register_id + buff[0] + buff[1] + buff[2]
   checksum = checksum ^ 0xFF
   if (checksum == buff[3])  // buff[3] là checksum byte
   ```
   
3. **Nếu checksum đúng:**
   - Gọi callback handler: `cmd_is_running.p_func(rx_data, 3)`
   - Reset: `cmd_is_running.id_register = REG_UNKNOWN`

### **BƯỚC 5: Xử lý dữ liệu (Callback Handler)**

**Ví dụ: bl0906_handle_current_rsp()**
```c
1. Chuyển đổi 3 bytes → u32: array_to_u24(par)
2. Tính toán giá trị:
   current = data * Vref / (12875 * Rl * Gain_i)
   current *= 1000  // A → mA
3. Xử lý current correction (nếu đang chạy)
4. Gọi callback: bl0906_update_energy(TYPE_CURRENT, current)
   → Application layer nhận được giá trị
```

**Ví dụ: bl0906_handle_voltage_rsp()**
```c
1. Chuyển đổi 3 bytes → u32
2. Tính toán:
   voltage = data * Vref * (Rf + Rv) / (13162 * Rv * Gain_v * 1000)
3. Gọi callback: bl0906_update_energy(TYPE_VOLTAGE, voltage)
```

## Sơ đồ Flow

```
Application
    ↓
bl0906_send_get_current()
    ↓
bl0906_read_register(I1_RMS)
    ↓
bl0906_push_wait_reg_rsp_to_fifo()
    ↓
FifoPush() → [I1_RMS, I2_RMS, I3_RMS] vào queue
    ↓
bl0906_proc() (gọi trong loop)
    ↓
bl0906_fifo_proc()
    ↓
FifoPop() → Lấy I1_RMS ra
    ↓
Gán callback: bl0906_handle_current_rsp
    ↓
Gửi UART: [0x35, I1_RMS]
    ↓
[CHỜ RESPONSE...]
    ↓
UART nhận data → bl0906_handle_serial_rx_message()
    ↓
Kiểm tra checksum
    ↓
Gọi callback: bl0906_handle_current_rsp(data)
    ↓
Tính toán giá trị current
    ↓
bl0906_update_energy(TYPE_CURRENT, value)
    ↓
Application callback nhận được giá trị
    ↓
Reset cmd_is_running → Lấy command tiếp theo từ FIFO
```

## Đặc điểm quan trọng

1. **FIFO Queue:** Quản lý nhiều command, xử lý tuần tự
2. **Timeout:** Mỗi command có timeout 100ms, nếu không nhận được response thì bỏ qua
3. **Callback Pattern:** Mỗi register có handler riêng để xử lý response
4. **Checksum:** Kiểm tra tính toàn vẹn dữ liệu
5. **Non-blocking:** Không block chờ response, dùng state machine

## Các hàm chính

| Hàm | Chức năng |
|-----|-----------|
| `bl0906_read_register()` | Push command vào FIFO |
| `bl0906_fifo_proc()` | Xử lý FIFO, gửi command |
| `bl0906_handle_serial_rx_message()` | Nhận và xử lý response |
| `bl0906_handle_*_rsp()` | Callback xử lý từng loại data |
| `bl0906_proc()` | Hàm chính gọi trong loop() |

