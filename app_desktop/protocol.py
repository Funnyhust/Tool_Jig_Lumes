import struct

class JigProtocol:
    HEADER = 0xAA
    
    CMD_ENABLE_ANALYZE = 0x01
    CMD_DISABLE_ANALYZE = 0x00
    CMD_READ_ALL_DATA = 0x10
    CMD_RELAY_CONTROL = 0x20 # 0x20 + Index
    CMD_BATCH_CONTROL = 0x30

    @staticmethod
    def pack(cmd, d1=0x00, d2=0x00):
        packet = bytearray([JigProtocol.HEADER, cmd, d1, d2])
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        return bytes(packet)

    @staticmethod
    def unpack_response(raw_bytes):
        if len(raw_bytes) < 5:
            return None
        
        header, cmd = raw_bytes[0], raw_bytes[1]
        if header != JigProtocol.HEADER:
            return None
            
        if cmd == JigProtocol.CMD_READ_ALL_DATA:
            if len(raw_bytes) < 75: # 1(H) + 1(C) + 72(D) + 1(CS)
                return None
            data = raw_bytes[2:74]
            checksum = raw_bytes[74]
            expected_cs = (sum(raw_bytes[:74])) & 0xFF
            if checksum != expected_cs:
                return None
                
            # Data structure: 4*V(8), 12*I(24), 12*P(24), 4*Gain(16)
            voltages = struct.unpack(">HHHH", data[0:8])
            currents = struct.unpack(">HHHHHHHHHHHH", data[8:32])
            powers = struct.unpack(">HHHHHHHHHHHH", data[32:56])
            gains = struct.unpack(">IIII", data[56:72])
            
            return {
                "type": "data",
                "voltages": [v / 10.0 for v in voltages],
                "currents": list(currents),
                "powers": [p / 10.0 for p in powers],
                "gains": list(gains)
            }
        else:
            d1, d2 = raw_bytes[2], raw_bytes[3]
            checksum = raw_bytes[4]
            expected_cs = (sum(raw_bytes[:4])) & 0xFF
            if checksum != expected_cs:
                return None
            return {
                "type": "ack",
                "cmd": cmd,
                "d1": d1,
                "d2": d2
            }
