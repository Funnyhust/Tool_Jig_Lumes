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
            self.log(f"Port {port} opened. Waiting for firmware events...")
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

    def is_healthy(self):
        if not self.running or not self.ser or not self.ser.is_open:
            return False
        if self.last_rx_time == 0:
            return None
        return (time.time() - self.last_rx_time) < 5.0

    def _receive_loop(self):
        buffer = bytearray()
        while self.running:
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    data = self.ser.read(waiting)
                    if not data:
                        continue
                    
                    buffer.extend(data)
                    
                    # Process buffer while it has enough data for at least a header + cmd
                    while len(buffer) >= 2:
                        # Find the first HEADER byte
                        if buffer[0] != JigProtocol.HEADER:
                            del buffer[0]
                            continue
                        
                        cmd = buffer[1]
                        
                        # Determine expected length
                        if cmd in (JigProtocol.CMD_TEST_START, JigProtocol.CMD_TEST_END, 
                                   JigProtocol.CMD_COMM_ERROR, JigProtocol.CMD_EEPROM_ERROR):
                            expected_len = 5
                        elif JigProtocol.CMD_CH_DATA_START <= cmd <= JigProtocol.CMD_CH_DATA_END:
                            expected_len = 24
                        elif cmd == JigProtocol.CMD_SUMMARY_FRAME:
                            expected_len = 108
                        else:
                            # Invalid CMD after HEADER, discard header and try to find next header
                            del buffer[0]
                            continue
                        
                        # If we don't have enough bytes for this CMD yet, wait for more data
                        if len(buffer) < expected_len:
                            break
                        
                        # Extract the full packet
                        packet_data = buffer[:expected_len]
                        
                        # Process packet
                        parsed = JigProtocol.unpack_response(packet_data)
                        if parsed:
                            self.last_rx_time = time.time()
                            self.callback(parsed)
                            # Remove processed packet from buffer
                            del buffer[:expected_len]
                        else:
                            # Log RAW hex for debugging
                            raw_hex = " ".join([f"{b:02X}" for b in packet_data[:16]])
                            self.log(f"PARSE FAIL: cmd=0x{cmd:02X} Raw={raw_hex}...")
                            # Only remove the header byte and try again to find next sync point
                            del buffer[0]
                else:
                    time.sleep(0.005) # Small sleep to yield CPU
            except Exception as e:
                self.log(f"Serial thread error: {e}")
                self.running = False
                break
        self.log("Serial receiver thread stopped.")
