import serial
import threading
import time
from protocol import JigProtocol

class SerialManager:
    def __init__(self, callback, log_callback=None):
        self.ser = None
        self.running = False
        self.callback = callback
        self.log_callback = log_callback
        self.thread = None
        self.last_rx_time = 0

    def log(self, message):
        if self.log_callback:
            self.log_callback(message)
        else:
            print(message)

    def connect(self, port, baudrate=115200):
        try:
            self.log(f"Opening port {port} at {baudrate} baud...")
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.running = True
            self.last_rx_time = 0
            self.thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.thread.start()
            self.log(f"Port {port} opened successfully.")
            return True
        except Exception as e:
            self.log(f"Error opening port: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.ser:
            self.ser.close()
            self.log("Port closed.")
        if self.thread:
            self.thread.join(timeout=1)

    def send_command(self, cmd, d1=0x00, d2=0x00):
        if self.ser and self.ser.is_open:
            packet = JigProtocol.pack(cmd, d1, d2)
            self.ser.write(packet)
            hex_data = " ".join([f"{b:02X}" for b in packet])
            self.log(f"TX: {hex_data}")
            return True
        return False

    def is_healthy(self):
        if not self.running or not self.ser or not self.ser.is_open:
            return False
        if self.last_rx_time == 0:
            return None
        return (time.time() - self.last_rx_time) < 3.0

    def _receive_loop(self):
        buffer = bytearray()
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    byte = self.ser.read(1)
                    if not byte: continue
                    
                    if len(buffer) == 0 and byte[0] != JigProtocol.HEADER:
                        continue
                    
                    buffer.extend(byte)
                    
                    if len(buffer) >= 2:
                        cmd = buffer[1]
                        expected_len = 5
                        if cmd == JigProtocol.CMD_READ_ALL_DATA:
                            expected_len = 75
                            
                        if len(buffer) == expected_len:
                            # Tắt dòng log RX để tránh lag UI nếu nhận liên tục giống docklight
                            # hex_rx = " ".join([f"{b:02X}" for b in buffer])
                            # self.log(f"RX: {hex_rx}")
                            
                            parsed = JigProtocol.unpack_response(buffer)
                            if parsed:
                                self.last_rx_time = time.time()
                                self.callback(parsed)
                            else:
                                self.log("RX: PARSE FAILED (Checksum Error?)")
                            buffer = bytearray()
                    
                    if len(buffer) > 100:
                        buffer = bytearray()
                else:
                    time.sleep(0.001)
            except Exception as e:
                self.log(f"Serial thread error: {e}")
                self.running = False
                break
        self.log("Serial receiver thread stopped.")
